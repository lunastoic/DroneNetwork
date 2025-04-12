package drone;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Random;
import java.util.Scanner;

/**
 * Main driver for the drone emergency simulation. 
 * It now loops over battery configurations and uses user-specified numbers for:
 *   - Total service nodes (max 100)
 *   - High-priority nodes to visit (max 20; selected based on highest weight)
 * 
 * Each service node is assigned a random weight (payload delivery amount). The highest weighted nodes
 * are considered high-priority. This simulation also incorporates payload drop-off: after a service node
 * is visited, its delivered weight is subtracted from the drone's payload for subsequent legs.
 *
 * The simulation outputs CSV summary statistics.
 */
public class DroneEmergencySim {
    private static final Random RAND = new Random();

    public static void main(String[] args) {
        Scanner scanner = new Scanner(System.in);

        // Simulation dimensions.
        System.out.print("Enter simulation width (e.g., 5000m): ");
        double width = scanner.nextDouble();

        System.out.print("Enter simulation length (e.g., 5000m): ");
        double length = scanner.nextDouble();

        System.out.print("Enter cruising altitude (e.g., 100m): ");
        double maxAltitude = scanner.nextDouble();
        if (maxAltitude < 0 || maxAltitude > 6000) {
            System.out.println("Error: Max altitude must be between 0 and 6000m. Adjusting...");
            maxAltitude = Math.max(0, Math.min(6000, maxAltitude));
        }
        if (maxAltitude < 50) {
            System.out.println("Warning: Max altitude below 50m may not clear obstacles.");
        }

        // Ask for number of service nodes (max 100)
        System.out.print("Enter number of service nodes (max 100): ");
        int numServiceNodes = scanner.nextInt();
        if (numServiceNodes > 100) {
            numServiceNodes = 100;
            System.out.println("Capping service nodes at 100.");
        }

        // Ask for number of high priority nodes (max 20)
        System.out.print("Enter number of high priority nodes to visit (max 20): ");
        int numHighPriority = scanner.nextInt();
        if (numHighPriority > 20) {
            numHighPriority = 20;
            System.out.println("Capping high priority nodes at 20.");
        }
        scanner.close();

        // Battery and payload configurations.
        int[] batteriesList = {1, 2};
        // For one battery: max payload = 40kg; for two batteries: max payload = 30kg.
        double[] payloadWeightsList = {40.0, 30.0};

        // Create CSV file for summary.
        String timestamp = LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss"));
        String csvFile = "results_summary_" + timestamp + ".csv";
        try (PrintWriter writer = new PrintWriter(new FileWriter(csvFile, true))) {
            writer.println("Scenario,Algorithm,Batteries,Payload,ServiceNodes,HighPriorityNodes,EnergyAvg (Wh),EnergyStd (Wh),TimeAvg (s),TimeStd (s),DistanceAvg (m),DistanceStd (m),PointsVisitedAvg,AdaptabilityAvg (%),DepotReturnsAvg,CostAvg ($),CostStd ($),CompTimeAvg (s),CompTimeStd (s)");
        } catch (IOException e) {
            System.out.println("Error creating CSV file: " + e.getMessage());
            return;
        }

        // We'll store one scenario's routes for visualization.
        List<SpatialNode> visGreedyRoute = null;
        List<SpatialNode> visMstRoute = null;
        Graph3D visGraph = null;

        // Loop over battery/payload configurations.
        for (int bIdx = 0; bIdx < batteriesList.length; bIdx++) {
            int battery = batteriesList[bIdx];
            double payload = payloadWeightsList[bIdx];
            String scenarioLabel = numServiceNodes + "sn_" + battery + "b_" + ((int) payload) + "kg";

            System.out.printf("Running scenario: %s%n", scenarioLabel);

            // Lists for metrics (for both algorithms).
            List<Double> greedyEnergies = new ArrayList<>();
            List<Double> greedyTimes = new ArrayList<>();
            List<Double> greedyDistances = new ArrayList<>();
            List<Integer> greedyPointsVisited = new ArrayList<>();
            List<Double> greedyAdaptabilities = new ArrayList<>();
            List<Integer> greedyDepotReturns = new ArrayList<>();
            List<Double> greedyCosts = new ArrayList<>();
            List<Double> greedyCompTimes = new ArrayList<>();

            List<Double> mstEnergies = new ArrayList<>();
            List<Double> mstTimes = new ArrayList<>();
            List<Double> mstDistances = new ArrayList<>();
            List<Integer> mstPointsVisited = new ArrayList<>();
            List<Double> mstAdaptabilities = new ArrayList<>();
            List<Integer> mstDepotReturns = new ArrayList<>();
            List<Double> mstCosts = new ArrayList<>();
            List<Double> mstCompTimes = new ArrayList<>();

            // Run multiple trials.
            int numTrials = 20; // You can also ask the user for number of trials.
            for (int trial = 1; trial <= numTrials; trial++) {
                Graph3D graph = new Graph3D(width, maxAltitude, length);
                DroneNode depot = new DroneNode(0, 0, 0);
                graph.addNode(depot);

                // Generate 'numServiceNodes' random service nodes.
                List<ServicePoint> allServicePoints = new ArrayList<>();
                for (int i = 0; i < numServiceNodes; i++) {
                    double x = RAND.nextDouble() * width;
                    double z = RAND.nextDouble() * length;
                    // Generate a random payload amount (for delivery) between, say, 1kg to 10kg.
                    double weight = RAND.nextDouble() * 9 + 1;
                    ServicePoint sp = new ServicePoint(x, 0, z, weight);
                    graph.addNode(sp);
                    allServicePoints.add(sp);
                }
                // Sort service nodes by weight in descending order.
                Collections.sort(allServicePoints, Comparator.comparingDouble(ServicePoint::getWeight));
                Collections.reverse(allServicePoints);
                // Select only the top 'numHighPriority' service nodes.
                List<ServicePoint> highPriorityNodes = new ArrayList<>();
                for (int i = 0; i < Math.min(numHighPriority, allServicePoints.size()); i++) {
                    highPriorityNodes.add(allServicePoints.get(i));
                }

                // Run Greedy algorithm.
                long startTime = System.nanoTime();
                List<SpatialNode> greedyRoute = Algorithms3D.planRoute(1, graph, depot, highPriorityNodes, maxAltitude, battery, payload);
                long compTime = System.nanoTime() - startTime;
                double greedyCompTime = compTime / 1e9;
                double[] gMetrics = Algorithms3D.calculateMetricsForRoute(greedyRoute, numHighPriority, payload);
                greedyEnergies.add(gMetrics[0]);
                greedyTimes.add(gMetrics[1]);
                greedyDistances.add(gMetrics[5]);
                greedyPointsVisited.add((int) gMetrics[2]);
                greedyAdaptabilities.add(gMetrics[3]);
                greedyDepotReturns.add((int) gMetrics[4]);
                greedyCosts.add(Algorithms3D.calculateMonetaryCost(gMetrics[0]));
                greedyCompTimes.add(greedyCompTime);

                // Run MST-TSP algorithm.
                startTime = System.nanoTime();
                List<SpatialNode> mstRoute = Algorithms3D.planRoute(3, graph, depot, highPriorityNodes, maxAltitude, battery, payload);
                compTime = System.nanoTime() - startTime;
                double mstCompTime = compTime / 1e9;
                double[] mMetrics = Algorithms3D.calculateMetricsForRoute(mstRoute, numHighPriority, payload);
                mstEnergies.add(mMetrics[0]);
                mstTimes.add(mMetrics[1]);
                mstDistances.add(mMetrics[5]);
                mstPointsVisited.add((int) mMetrics[2]);
                mstAdaptabilities.add(mMetrics[3]);
                mstDepotReturns.add((int) mMetrics[4]);
                mstCosts.add(Algorithms3D.calculateMonetaryCost(mMetrics[0]));
                mstCompTimes.add(mstCompTime);

                // For visualization, pick a representative trial.
                if (battery == batteriesList[0] && payload == payloadWeightsList[0] && trial == numTrials) {
                    visGreedyRoute = greedyRoute;
                    visMstRoute = mstRoute;
                    visGraph = graph;
                }
            } // End trials

            // Compute summary statistics for each metric and write to CSV.
            double[] greedyEnergyStats = computeStats(greedyEnergies);
            double[] greedyTimeStats = computeStats(greedyTimes);
            double[] greedyDistanceStats = computeStats(greedyDistances);
            double[] greedyPointsStats = computeStatsInt(greedyPointsVisited);
            double[] greedyAdaptStats = computeStats(greedyAdaptabilities);
            double[] greedyDepotStats = computeStatsInt(greedyDepotReturns);
            double[] greedyCostStats = computeStats(greedyCosts);
            double[] greedyCompTimeStats = computeStats(greedyCompTimes);

            double[] mstEnergyStats = computeStats(mstEnergies);
            double[] mstTimeStats = computeStats(mstTimes);
            double[] mstDistanceStats = computeStats(mstDistances);
            double[] mstPointsStats = computeStatsInt(mstPointsVisited);
            double[] mstAdaptStats = computeStats(mstAdaptabilities);
            double[] mstDepotStats = computeStatsInt(mstDepotReturns);
            double[] mstCostStats = computeStats(mstCosts);
            double[] mstCompTimeStats = computeStats(mstCompTimes);

            writeSummary(csvFile, scenarioLabel, "Greedy", battery, payload, numHighPriority,
                    greedyEnergyStats, greedyTimeStats, greedyDistanceStats, greedyPointsStats,
                    greedyAdaptStats, greedyDepotStats, greedyCostStats, greedyCompTimeStats);
            writeSummary(csvFile, scenarioLabel, "MST-TSP", battery, payload, numHighPriority,
                    mstEnergyStats, mstTimeStats, mstDistanceStats, mstPointsStats,
                    mstAdaptStats, mstDepotStats, mstCostStats, mstCompTimeStats);

            System.out.println("Summary for " + scenarioLabel + " (Greedy):");
            System.out.printf("  Energy: %.2f ± %.2f Wh, Time: %.2f ± %.2f s, Distance: %.2f ± %.2f m, Points: %.2f, Adapt: %.2f%%, Depot Returns: %.2f, Cost: $%.2f ± $%.2f, CompTime: %.2f ± %.2f s%n",
                    greedyEnergyStats[0], greedyEnergyStats[1],
                    greedyTimeStats[0], greedyTimeStats[1],
                    greedyDistanceStats[0], greedyDistanceStats[1],
                    greedyPointsStats[0],
                    greedyAdaptStats[0],
                    greedyDepotStats[0],
                    greedyCostStats[0], greedyCostStats[1],
                    greedyCompTimeStats[0], greedyCompTimeStats[1]);
            System.out.println("Summary for " + scenarioLabel + " (MST-TSP):");
            System.out.printf("  Energy: %.2f ± %.2f Wh, Time: %.2f ± %.2f s, Distance: %.2f ± %.2f m, Points: %.2f, Adapt: %.2f%%, Depot Returns: %.2f, Cost: $%.2f ± $%.2f, CompTime: %.2f ± %.2f s%n",
                    mstEnergyStats[0], mstEnergyStats[1],
                    mstTimeStats[0], mstTimeStats[1],
                    mstDistanceStats[0], mstDistanceStats[1],
                    mstPointsStats[0],
                    mstAdaptStats[0],
                    mstDepotStats[0],
                    mstCostStats[0], mstCostStats[1],
                    mstCompTimeStats[0], mstCompTimeStats[1]);
        }
        System.out.println("All simulations completed. Summary results saved to " + csvFile);

        if (visGraph != null && visGreedyRoute != null && visMstRoute != null) {
            System.out.println("Launching visualizer for scenario: " + visGraph.getWidth() + " / " + visGraph.getAltitude() + " / " + visGraph.getLength());
            DroneNetworkVisualization3D.launchVisualizer(visGraph, visGreedyRoute, visMstRoute);
        }
    }

    private static double[] computeStats(List<Double> values) {
        double sum = 0;
        for (double v : values) sum += v;
        double mean = sum / values.size();
        double sumSq = 0;
        for (double v : values) {
            double diff = v - mean;
            sumSq += diff * diff;
        }
        double stdDev = Math.sqrt(sumSq / values.size());
        return new double[]{mean, stdDev};
    }

    private static double[] computeStatsInt(List<Integer> values) {
        double sum = 0;
        for (int v : values) sum += v;
        double mean = sum / values.size();
        double sumSq = 0;
        for (int v : values) {
            double diff = v - mean;
            sumSq += diff * diff;
        }
        double stdDev = Math.sqrt(sumSq / values.size());
        return new double[]{mean, stdDev};
    }

    private static void writeSummary(String csvFile, String scenario, String algorithm, int batteries, double payload, int highPriorityNodes,
                                     double[] energyStats, double[] timeStats, double[] distanceStats, double[] pointsStats,
                                     double[] adaptStats, double[] depotStats, double[] costStats, double[] compTimeStats) {
        try (PrintWriter writer = new PrintWriter(new FileWriter(csvFile, true))) {
            writer.printf("%s,%s,%d,%.1f,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f%n",
                    scenario, algorithm, batteries, payload, highPriorityNodes,
                    highPriorityNodes,
                    energyStats[0], energyStats[1],
                    timeStats[0], timeStats[1],
                    distanceStats[0], distanceStats[1],
                    pointsStats[0],
                    adaptStats[0],
                    depotStats[0],
                    costStats[0], costStats[1],
                    compTimeStats[0], compTimeStats[1]);
        } catch (IOException e) {
            System.out.println("Error writing summary to CSV: " + e.getMessage());
        }
    }
}

