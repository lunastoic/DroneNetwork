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
 * 
 * The program:
 *  - Prompts for simulation dimensions, number of sensor nodes, and number of high-priority nodes to visit.
 *  - Generates a sensor network as a graph and selects the top high-priority nodes.
 *  - Simulates the drone completing one loop (visiting all high-priority nodes) using both:
 *     (1) Greedy algorithm and (2) MST-based TSP Approximation.
 *  - Collects metrics such as Energy Consumed (Wh), Flight Time (s), Total Distance (m), Depot Returns,
 *    and Battery Efficiency (% consumed and left), as well as computational time for route planning.
 *  - Runs 40 trials per scenario (for 1 battery and 2 batteries, each with Greedy and MST approaches),
 *    aggregates the data (using averages and standard deviations), and outputs the results to a CSV file.
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

        // Service nodes generation parameters.
        System.out.print("Enter number of service nodes (max 100): ");
        int numServiceNodes = scanner.nextInt();
        if (numServiceNodes > 100) {
            numServiceNodes = 100;
            System.out.println("Capping service nodes at 100.");
        }

        System.out.print("Enter number of high priority nodes to visit (max 20): ");
        int numHighPriority = scanner.nextInt();
        if (numHighPriority > 20) {
            numHighPriority = 20;
            System.out.println("Capping high priority nodes at 20.");
        }
        scanner.close();

        // Battery and payload configurations.
        int[] batteriesList = {1, 2};
        // Use a fixed payload of 5 kg.
        double[] payloadWeightsList = {5.0};

        // Create CSV file for summary results.
        String timestamp = LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss"));
        String csvFile = "results_summary_" + timestamp + ".csv";
        try (PrintWriter writer = new PrintWriter(new FileWriter(csvFile, true))) {
            writer.println("Scenario,Algorithm,Batteries,Payload,ServiceNodes,HighPriorityNodes,EnergyAvg (Wh),EnergyStd (Wh),TimeAvg (s),TimeStd (s),DistanceAvg (m),DistanceStd (m),PointsVisitedAvg,AdaptabilityAvg (%),DepotReturnsAvg,CostAvg ($),CostStd ($),CompTimeAvg (s),CompTimeStd (s),BatteryConsumedAvg (%),BatteryLeftAvg (%)");
        } catch (IOException e) {
            System.out.println("Error creating CSV file: " + e.getMessage());
            return;
        }

        // For visualization, store one scenario's routes.
        List<SpatialNode> visGreedyRoute = null;
        List<SpatialNode> visMstRoute = null;
        Graph3D visGraph = null;

        // We'll run 40 trials per scenario.
        int numTrials = 40;

        // Initialize lists to store aggregated metrics for each scenario and algorithm.
        // For each metric we create separate lists per algorithm scenario.
        // We use one payload value so we have four scenarios overall.
        for (int battery : batteriesList) {
            for (double payload : payloadWeightsList) {
                String scenarioLabel = numServiceNodes + "sn_" + battery + "b_" + ((int) payload) + "kg";
                System.out.printf("Running scenario: %s%n", scenarioLabel);

                // Metric accumulators for the Greedy algorithm.
                List<Double> greedyEnergies = new ArrayList<>();
                List<Double> greedyTimes = new ArrayList<>();
                List<Double> greedyDistances = new ArrayList<>();
                List<Integer> greedyPointsVisited = new ArrayList<>();
                List<Double> greedyAdaptabilities = new ArrayList<>();
                List<Integer> greedyDepotReturns = new ArrayList<>();
                List<Double> greedyCosts = new ArrayList<>();
                List<Double> greedyCompTimes = new ArrayList<>();
                List<Double> greedyBatteryConsumed = new ArrayList<>();
                List<Double> greedyBatteryLeft = new ArrayList<>();

                // Metric accumulators for the MST-TSP algorithm.
                List<Double> mstEnergies = new ArrayList<>();
                List<Double> mstTimes = new ArrayList<>();
                List<Double> mstDistances = new ArrayList<>();
                List<Integer> mstPointsVisited = new ArrayList<>();
                List<Double> mstAdaptabilities = new ArrayList<>();
                List<Integer> mstDepotReturns = new ArrayList<>();
                List<Double> mstCosts = new ArrayList<>();
                List<Double> mstCompTimes = new ArrayList<>();
                List<Double> mstBatteryConsumed = new ArrayList<>();
                List<Double> mstBatteryLeft = new ArrayList<>();

                // Run multiple trials.
                for (int trial = 1; trial <= numTrials; trial++) {
                    // Build the simulation graph.
                    Graph3D graph = new Graph3D(width, maxAltitude, length);
                    DroneNode depot = new DroneNode(0, 0, 0);
                    graph.addNode(depot);

                    // Generate sensor nodes.
                    List<ServicePoint> servicePoints = new ArrayList<>();
                    for (int i = 0; i < numServiceNodes; i++) {
                        double x = RAND.nextDouble() * width;
                        double z = RAND.nextDouble() * length;
                        double weight = RAND.nextDouble() * 99 + 1; // Random priority 1-100.
                        ServicePoint sp = new ServicePoint(x, 0, z, weight);
                        graph.addNode(sp);
                        servicePoints.add(sp);
                    }
                    // Sort service points by descending weight and select top high-priority nodes.
                    Collections.sort(servicePoints, Comparator.comparingDouble(ServicePoint::getWeight));
                    Collections.reverse(servicePoints);
                    List<ServicePoint> highPriorityPoints = new ArrayList<>();
                    for (int i = 0; i < Math.min(numHighPriority, servicePoints.size()); i++) {
                        highPriorityPoints.add(servicePoints.get(i));
                    }

                    // Determine battery capacity.
                    double batteryCapacity = (battery == 1) ? 1984.5 : 3969.0;

                    // --- Run Greedy ---
                    long startTime = System.nanoTime();
                    List<SpatialNode> greedyRoute = Algorithms3D.planRoute(1, graph, depot, highPriorityPoints, maxAltitude, battery, payload);
                    long endTime = System.nanoTime();
                    double greedyCompTime = (endTime - startTime) / 1e9;
                    double[] gMetrics = Algorithms3D.calculateMetricsForRoute(greedyRoute, numServiceNodes, batteryCapacity);
                    // Metrics indices: [0]=EnergyAvg, [1]=Time, [2]=PointsVisited, [3]=Adaptability, [4]=DepotReturns, [5]=Distance, [6]=BatteryConsumedAvg, [7]=BatteryLeftAvg
                    greedyEnergies.add(gMetrics[0]);
                    greedyTimes.add(gMetrics[1]);
                    greedyDistances.add(gMetrics[5]);
                    greedyPointsVisited.add((int) gMetrics[2]);
                    greedyAdaptabilities.add(gMetrics[3]);
                    greedyDepotReturns.add((int) gMetrics[4]);
                    greedyCosts.add(Algorithms3D.calculateMonetaryCost(gMetrics[0]));
                    greedyCompTimes.add(greedyCompTime);
                    greedyBatteryConsumed.add(gMetrics[6]);
                    greedyBatteryLeft.add(gMetrics[7]);

                    // --- Run MST-TSP Approximation ---
                    startTime = System.nanoTime();
                    List<SpatialNode> mstRoute = Algorithms3D.planRoute(3, graph, depot, highPriorityPoints, maxAltitude, battery, payload);
                    endTime = System.nanoTime();
                    double mstCompTime = (endTime - startTime) / 1e9;
                    double[] mMetrics = Algorithms3D.calculateMetricsForRoute(mstRoute, numServiceNodes, batteryCapacity);
                    mstEnergies.add(mMetrics[0]);
                    mstTimes.add(mMetrics[1]);
                    mstDistances.add(mMetrics[5]);
                    mstPointsVisited.add((int) mMetrics[2]);
                    mstAdaptabilities.add(mMetrics[3]);
                    mstDepotReturns.add((int) mMetrics[4]);
                    mstCosts.add(Algorithms3D.calculateMonetaryCost(mMetrics[0]));
                    mstCompTimes.add(mstCompTime);
                    mstBatteryConsumed.add(mMetrics[6]);
                    mstBatteryLeft.add(mMetrics[7]);

                    // For visualization, store one scenario (first battery configuration, last trial).
                    if (battery == batteriesList[0] && payload == payloadWeightsList[0] && trial == numTrials) {
                        visGreedyRoute = greedyRoute;
                        visMstRoute = mstRoute;
                        visGraph = graph;
                    }
                } // End trial loop

                // Compute aggregated statistics (mean and std) for each metric.
                double[] greedyEnergyStats = computeStats(greedyEnergies);
                double[] greedyTimeStats = computeStats(greedyTimes);
                double[] greedyDistanceStats = computeStats(greedyDistances);
                double[] greedyPointsStats = computeStatsInt(greedyPointsVisited);
                double[] greedyAdaptStats = computeStats(greedyAdaptabilities);
                double[] greedyDepotStats = computeStatsInt(greedyDepotReturns);
                double[] greedyCostStats = computeStats(greedyCosts);
                double[] greedyCompTimeStats = computeStats(greedyCompTimes);
                double[] greedyBatteryConsumedStats = computeStats(greedyBatteryConsumed);
                double[] greedyBatteryLeftStats = computeStats(greedyBatteryLeft);

                double[] mstEnergyStats = computeStats(mstEnergies);
                double[] mstTimeStats = computeStats(mstTimes);
                double[] mstDistanceStats = computeStats(mstDistances);
                double[] mstPointsStats = computeStatsInt(mstPointsVisited);
                double[] mstAdaptStats = computeStats(mstAdaptabilities);
                double[] mstDepotStats = computeStatsInt(mstDepotReturns);
                double[] mstCostStats = computeStats(mstCosts);
                double[] mstCompTimeStats = computeStats(mstCompTimes);
                double[] mstBatteryConsumedStats = computeStats(mstBatteryConsumed);
                double[] mstBatteryLeftStats = computeStats(mstBatteryLeft);

                // Write aggregated results to CSV.
                writeSummary(csvFile, scenarioLabel, "Greedy", battery, payload, numServiceNodes,
                        greedyEnergyStats, greedyTimeStats, greedyDistanceStats, greedyPointsStats,
                        greedyAdaptStats, greedyDepotStats, greedyCostStats, greedyCompTimeStats,
                        greedyBatteryConsumedStats[0], greedyBatteryLeftStats[0]);
                writeSummary(csvFile, scenarioLabel, "MST-TSP", battery, payload, numServiceNodes,
                        mstEnergyStats, mstTimeStats, mstDistanceStats, mstPointsStats,
                        mstAdaptStats, mstDepotStats, mstCostStats, mstCompTimeStats,
                        mstBatteryConsumedStats[0], mstBatteryLeftStats[0]);

                // Print summary to console.
                System.out.println("Summary for " + scenarioLabel + " (Greedy):");
                System.out.printf("  Energy: %.2f ± %.2f Wh, Time: %.2f ± %.2f s, Distance: %.2f ± %.2f m, Points: %.2f, Adapt: %.2f%%, Depot Returns: %.2f, Cost: $%.2f ± $%.2f, CompTime: %.2f ± %.2f s, Battery Used: %.2f%%, Battery Left: %.2f%%%n",
                        greedyEnergyStats[0], greedyEnergyStats[1],
                        greedyTimeStats[0], greedyTimeStats[1],
                        greedyDistanceStats[0], greedyDistanceStats[1],
                        greedyPointsStats[0],
                        greedyAdaptStats[0],
                        greedyDepotStats[0],
                        greedyCostStats[0], greedyCostStats[1],
                        greedyCompTimeStats[0], greedyCompTimeStats[1],
                        greedyBatteryConsumedStats[0], greedyBatteryLeftStats[0]);
                System.out.println("Summary for " + scenarioLabel + " (MST-TSP):");
                System.out.printf("  Energy: %.2f ± %.2f Wh, Time: %.2f ± %.2f s, Distance: %.2f ± %.2f m, Points: %.2f, Adapt: %.2f%%, Depot Returns: %.2f, Cost: $%.2f ± $%.2f, CompTime: %.2f ± %.2f s, Battery Used: %.2f%%, Battery Left: %.2f%%%n",
                        mstEnergyStats[0], mstEnergyStats[1],
                        mstTimeStats[0], mstTimeStats[1],
                        mstDistanceStats[0], mstDistanceStats[1],
                        mstPointsStats[0],
                        mstAdaptStats[0],
                        mstDepotStats[0],
                        mstCostStats[0], mstCostStats[1],
                        mstCompTimeStats[0], mstCompTimeStats[1],
                        mstBatteryConsumedStats[0], mstBatteryLeftStats[0]);
            }
        }
        System.out.println("All simulations completed. Summary results saved to " + csvFile);

        if (visGraph != null && visGreedyRoute != null && visMstRoute != null) {
            System.out.println("Launching visualizer for scenario: " 
                    + visGraph.getWidth() + " / " 
                    + visGraph.getAltitude() + " / " 
                    + visGraph.getLength());
            DroneNetworkVisualization3D.launchVisualizer(visGraph, visGreedyRoute, visMstRoute);
        }
    }

    // Utility: Compute mean and standard deviation for a list of doubles.
    private static double[] computeStats(List<Double> values) {
        double sum = 0;
        for (double v : values) {
            sum += v;
        }
        double mean = sum / values.size();
        double sumSq = 0;
        for (double v : values) {
            double diff = v - mean;
            sumSq += diff * diff;
        }
        double stdDev = Math.sqrt(sumSq / values.size());
        return new double[]{mean, stdDev};
    }

    // Utility: Compute mean and standard deviation for a list of integers.
    private static double[] computeStatsInt(List<Integer> values) {
        double sum = 0;
        for (int v : values) {
            sum += v;
        }
        double mean = sum / values.size();
        double sumSq = 0;
        for (int v : values) {
            double diff = v - mean;
            sumSq += diff * diff;
        }
        double stdDev = Math.sqrt(sumSq / values.size());
        return new double[]{mean, stdDev};
    }

    private static void writeSummary(String csvFile, String scenario, String algorithm, int batteries, double payload, int serviceNodes,
                                     double[] energyStats, double[] timeStats, double[] distanceStats, double[] pointsStats,
                                     double[] adaptStats, double[] depotStats, double[] costStats, double[] compTimeStats,
                                     double batteryConsumed, double batteryLeft) {
        try (PrintWriter writer = new PrintWriter(new FileWriter(csvFile, true))) {
            writer.printf("%s,%s,%d,%.1f,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%n",
                    scenario, algorithm, batteries, payload, serviceNodes, serviceNodes,
                    energyStats[0], energyStats[1],
                    timeStats[0], timeStats[1],
                    distanceStats[0], distanceStats[1],
                    pointsStats[0],
                    adaptStats[0],
                    depotStats[0],
                    costStats[0], costStats[1],
                    compTimeStats[0], compTimeStats[1],
                    batteryConsumed, batteryLeft);
        } catch (IOException e) {
            System.out.println("Error writing summary to CSV: " + e.getMessage());
        }
    }
}
