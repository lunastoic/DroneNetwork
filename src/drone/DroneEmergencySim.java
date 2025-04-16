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
 * This version arranges overall and mission metrics side by side in the CSV output,
 * and correctly passes single overall battery values rather than arrays
 * to match the writeSummary(...) method signature.
 */
public class DroneEmergencySim {
    private static final Random RAND = new Random();

    public static void main(String[] args) {
        Scanner scanner = new Scanner(System.in);

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

        int[] batteriesList = {1, 2};
        double[] payloadWeightsList = {5.0};

        // Create CSV file for summary results.
        String timestamp = LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss"));
        String csvFile = "results_summary_" + timestamp + ".csv";
        try (PrintWriter writer = new PrintWriter(new FileWriter(csvFile, true))) {
            // CSV header placing overall and mission metrics side by side
            writer.println("Scenario,Algorithm,Batteries,Payload,ServiceNodes,HighPriorityNodes," +
                    "OverallEnergyAvg (Wh),OverallEnergyStd (Wh)," +
                    "OverallTimeAvg (s),OverallTimeStd (s)," +
                    "MissionTimeAvg (s),MissionTimeStd (s)," +
                    "OverallBatteryConsumedAvg (%),OverallBatteryLeftAvg (%)," +
                    "MissionBatteryConsumedAvg (%),MissionBatteryConsumedStd (%),MissionBatteryLeftAvg (%),MissionBatteryLeftStd (%)," +
                    "DistanceAvg (m),DistanceStd (m),PointsVisitedAvg,AdaptabilityAvg (%),DepotReturnsAvg," +
                    "CostAvg ($),CostStd ($),CompTimeAvg (s),CompTimeStd (s)");
        } catch (IOException e) {
            System.out.println("Error creating CSV file: " + e.getMessage());
            return;
        }

        List<SpatialNode> visGreedyRoute = null;
        List<SpatialNode> visMstRoute = null;
        Graph3D visGraph = null;

        int numTrials = 40;

        // Accumulators for metrics. (We'll reset them per scenario, so these top-level lists
        // might be optional if you only store them in scenario-lists below.)

        for (int battery : batteriesList) {
            for (double payload : payloadWeightsList) {
                String scenarioLabel = numServiceNodes + "sn_" + battery + "b_" + ((int) payload) + "kg";
                System.out.printf("Running scenario: %s%n", scenarioLabel);

                // Per-scenario accumulators for Greedy
                List<Double> scenarioGreedyEnergies = new ArrayList<>();
                List<Double> scenarioGreedyTimes = new ArrayList<>();
                List<Double> scenarioGreedyDistances = new ArrayList<>();
                List<Integer> scenarioGreedyPointsVisited = new ArrayList<>();
                List<Double> scenarioGreedyAdaptabilities = new ArrayList<>();
                List<Integer> scenarioGreedyDepotReturns = new ArrayList<>();
                List<Double> scenarioGreedyCosts = new ArrayList<>();
                List<Double> scenarioGreedyCompTimes = new ArrayList<>();
                List<Double> scenarioGreedyBatteryConsumed = new ArrayList<>();
                List<Double> scenarioGreedyBatteryLeft = new ArrayList<>();
                List<Double> scenarioGreedyMissionTimes = new ArrayList<>();
                List<Double> scenarioGreedyMissionBatteryConsumed = new ArrayList<>();
                List<Double> scenarioGreedyMissionBatteryLeft = new ArrayList<>();

                // Per-scenario accumulators for MST
                List<Double> scenarioMstEnergies = new ArrayList<>();
                List<Double> scenarioMstTimes = new ArrayList<>();
                List<Double> scenarioMstDistances = new ArrayList<>();
                List<Integer> scenarioMstPointsVisited = new ArrayList<>();
                List<Double> scenarioMstAdaptabilities = new ArrayList<>();
                List<Integer> scenarioMstDepotReturns = new ArrayList<>();
                List<Double> scenarioMstCosts = new ArrayList<>();
                List<Double> scenarioMstCompTimes = new ArrayList<>();
                List<Double> scenarioMstBatteryConsumed = new ArrayList<>();
                List<Double> scenarioMstBatteryLeft = new ArrayList<>();
                List<Double> scenarioMstMissionTimes = new ArrayList<>();
                List<Double> scenarioMstMissionBatteryConsumed = new ArrayList<>();
                List<Double> scenarioMstMissionBatteryLeft = new ArrayList<>();

                // Run multiple trials.
                for (int trial = 1; trial <= numTrials; trial++) {
                    Graph3D graph = new Graph3D(width, maxAltitude, length);
                    DroneNode depot = new DroneNode(0, 0, 0);
                    graph.addNode(depot);

                    List<ServicePoint> servicePoints = new ArrayList<>();
                    for (int i = 0; i < numServiceNodes; i++) {
                        double x = RAND.nextDouble() * width;
                        double z = RAND.nextDouble() * length;
                        double weight = RAND.nextDouble() * 99 + 1;
                        ServicePoint sp = new ServicePoint(x, 0, z, weight);
                        graph.addNode(sp);
                        servicePoints.add(sp);
                    }
                    // Sort by descending weight to pick high priority
                    Collections.sort(servicePoints, Comparator.comparingDouble(ServicePoint::getWeight));
                    Collections.reverse(servicePoints);
                    List<ServicePoint> highPriorityPoints = new ArrayList<>();
                    for (int i = 0; i < Math.min(numHighPriority, servicePoints.size()); i++) {
                        highPriorityPoints.add(servicePoints.get(i));
                    }

                    double batteryCapacity = (battery == 1) ? 1984.5 : 3969.0;

                    // --- Run Greedy ---
                    long startTime = System.nanoTime();
                    List<SpatialNode> greedyRoute = Algorithms3D.planRoute(1, graph, depot, highPriorityPoints, maxAltitude, battery, payload);
                    long endTime = System.nanoTime();
                    double greedyCompTime = (endTime - startTime) / 1e9;
                    double[] gMetrics = Algorithms3D.calculateMetricsForRoute(greedyRoute, numServiceNodes, batteryCapacity);
                    scenarioGreedyEnergies.add(gMetrics[0]);
                    scenarioGreedyTimes.add(gMetrics[1]);
                    scenarioGreedyDistances.add(gMetrics[5]);
                    scenarioGreedyPointsVisited.add((int) gMetrics[2]);
                    scenarioGreedyAdaptabilities.add(gMetrics[3]);
                    scenarioGreedyDepotReturns.add((int) gMetrics[4]);
                    scenarioGreedyCosts.add(Algorithms3D.calculateMonetaryCost(gMetrics[0]));
                    scenarioGreedyCompTimes.add(greedyCompTime);
                    scenarioGreedyBatteryConsumed.add(gMetrics[6]);
                    scenarioGreedyBatteryLeft.add(gMetrics[7]);
                    scenarioGreedyMissionTimes.add(gMetrics[8]);
                    scenarioGreedyMissionBatteryConsumed.add(gMetrics[9]);
                    scenarioGreedyMissionBatteryLeft.add(gMetrics[10]);

                    // --- Run MST-TSP ---
                    startTime = System.nanoTime();
                    List<SpatialNode> mstRoute = Algorithms3D.planRoute(3, graph, depot, highPriorityPoints, maxAltitude, battery, payload);
                    endTime = System.nanoTime();
                    double mstCompTime = (endTime - startTime) / 1e9;
                    double[] mMetrics = Algorithms3D.calculateMetricsForRoute(mstRoute, numServiceNodes, batteryCapacity);
                    scenarioMstEnergies.add(mMetrics[0]);
                    scenarioMstTimes.add(mMetrics[1]);
                    scenarioMstDistances.add(mMetrics[5]);
                    scenarioMstPointsVisited.add((int) mMetrics[2]);
                    scenarioMstAdaptabilities.add(mMetrics[3]);
                    scenarioMstDepotReturns.add((int) mMetrics[4]);
                    scenarioMstCosts.add(Algorithms3D.calculateMonetaryCost(mMetrics[0]));
                    scenarioMstCompTimes.add(mstCompTime);
                    scenarioMstBatteryConsumed.add(mMetrics[6]);
                    scenarioMstBatteryLeft.add(mMetrics[7]);
                    scenarioMstMissionTimes.add(mMetrics[8]);
                    scenarioMstMissionBatteryConsumed.add(mMetrics[9]);
                    scenarioMstMissionBatteryLeft.add(mMetrics[10]);

                    // For visualization
                    if (battery == batteriesList[0] && payload == payloadWeightsList[0] && trial == numTrials) {
                        visGreedyRoute = greedyRoute;
                        visMstRoute = mstRoute;
                        visGraph = graph;
                    }
                } // End trial loop

                // Compute aggregated stats
                double[] greedyEnergyStats = computeStats(scenarioGreedyEnergies);
                double[] greedyTimeStats = computeStats(scenarioGreedyTimes);
                double[] greedyDistanceStats = computeStats(scenarioGreedyDistances);
                double[] greedyPointsStats = computeStatsInt(scenarioGreedyPointsVisited);
                double[] greedyAdaptStats = computeStats(scenarioGreedyAdaptabilities);
                double[] greedyDepotStats = computeStatsInt(scenarioGreedyDepotReturns);
                double[] greedyCostStats = computeStats(scenarioGreedyCosts);
                double[] greedyCompTimeStats = computeStats(scenarioGreedyCompTimes);
                double[] greedyBatteryConsumedStats = computeStats(scenarioGreedyBatteryConsumed);  // [mean, std]
                double[] greedyBatteryLeftStats = computeStats(scenarioGreedyBatteryLeft);          // [mean, std]
                double[] greedyMissionTimeStats = computeStats(scenarioGreedyMissionTimes);
                double[] greedyMissionBatteryConsumedStats = computeStats(scenarioGreedyMissionBatteryConsumed);
                double[] greedyMissionBatteryLeftStats = computeStats(scenarioGreedyMissionBatteryLeft);

                double[] mstEnergyStats = computeStats(scenarioMstEnergies);
                double[] mstTimeStats = computeStats(scenarioMstTimes);
                double[] mstDistanceStats = computeStats(scenarioMstDistances);
                double[] mstPointsStats = computeStatsInt(scenarioMstPointsVisited);
                double[] mstAdaptStats = computeStats(scenarioMstAdaptabilities);
                double[] mstDepotStats = computeStatsInt(scenarioMstDepotReturns);
                double[] mstCostStats = computeStats(scenarioMstCosts);
                double[] mstCompTimeStats = computeStats(scenarioMstCompTimes);
                double[] mstBatteryConsumedStats = computeStats(scenarioMstBatteryConsumed); // [mean, std]
                double[] mstBatteryLeftStats = computeStats(scenarioMstBatteryLeft);         // [mean, std]
                double[] mstMissionTimeStats = computeStats(scenarioMstMissionTimes);
                double[] mstMissionBatteryConsumedStats = computeStats(scenarioMstMissionBatteryConsumed);
                double[] mstMissionBatteryLeftStats = computeStats(scenarioMstMissionBatteryLeft);

                // Now we call writeSummary(...) with single MEAN values for overall battery consumed/left:
                writeSummary(csvFile, scenarioLabel, "Greedy", battery, payload, numServiceNodes,
                        greedyEnergyStats, greedyTimeStats,
                        greedyMissionTimeStats,
                        // pass the mean for overall battery consumed/left
                        greedyBatteryConsumedStats[0], // overall consumed (mean)
                        greedyBatteryLeftStats[0],     // overall left (mean)
                        greedyMissionBatteryConsumedStats, // mission consumed array
                        greedyMissionBatteryLeftStats,     // mission left array
                        greedyDistanceStats, greedyPointsStats, greedyAdaptStats, greedyDepotStats,
                        greedyCostStats, greedyCompTimeStats);

                // Do the same for MST
                writeSummary(csvFile, scenarioLabel, "MST-TSP", battery, payload, numServiceNodes,
                        mstEnergyStats, mstTimeStats,
                        mstMissionTimeStats,
                        // pass the mean for overall battery consumed/left
                        mstBatteryConsumedStats[0], // overall consumed (mean)
                        mstBatteryLeftStats[0],     // overall left (mean)
                        mstMissionBatteryConsumedStats, // mission consumed array
                        mstMissionBatteryLeftStats,     // mission left array
                        mstDistanceStats, mstPointsStats, mstAdaptStats, mstDepotStats,
                        mstCostStats, mstCompTimeStats);

                // Print summary to console
                System.out.println("Summary for " + scenarioLabel + " (Greedy):");
                System.out.printf("  Overall Energy: %.2f ± %.2f Wh, Overall Time: %.2f ± %.2f s, " +
                                "Mission Time: %.2f ± %.2f s, Overall Battery Used: %.2f%%, Overall Battery Left: %.2f%%, " +
                                "Mission Battery Used: %.2f%% ± %.2f%%, Mission Battery Left: %.2f%% ± %.2f%%, " +
                                "Distance: %.2f ± %.2f m, Points: %.2f, Adapt: %.2f%%, Depot Returns: %.2f, " +
                                "Cost: $%.2f ± $%.2f, CompTime: %.2f ± %.2f s%n",
                        greedyEnergyStats[0], greedyEnergyStats[1],
                        greedyTimeStats[0], greedyTimeStats[1],
                        greedyMissionTimeStats[0], greedyMissionTimeStats[1],
                        greedyBatteryConsumedStats[0], greedyBatteryLeftStats[0],
                        greedyMissionBatteryConsumedStats[0], greedyMissionBatteryConsumedStats[1],
                        greedyMissionBatteryLeftStats[0], greedyMissionBatteryLeftStats[1],
                        greedyDistanceStats[0], greedyDistanceStats[1],
                        greedyPointsStats[0],
                        greedyAdaptStats[0],
                        greedyDepotStats[0],
                        greedyCostStats[0], greedyCostStats[1],
                        greedyCompTimeStats[0], greedyCompTimeStats[1]);

                System.out.println("Summary for " + scenarioLabel + " (MST-TSP):");
                System.out.printf("  Overall Energy: %.2f ± %.2f Wh, Overall Time: %.2f ± %.2f s, " +
                                "Mission Time: %.2f ± %.2f s, Overall Battery Used: %.2f%%, Overall Battery Left: %.2f%%, " +
                                "Mission Battery Used: %.2f%% ± %.2f%%, Mission Battery Left: %.2f%% ± %.2f%%, " +
                                "Distance: %.2f ± %.2f m, Points: %.2f, Adapt: %.2f%%, Depot Returns: %.2f, " +
                                "Cost: $%.2f ± $%.2f, CompTime: %.2f ± %.2f s%n",
                        mstEnergyStats[0], mstEnergyStats[1],
                        mstTimeStats[0], mstTimeStats[1],
                        mstMissionTimeStats[0], mstMissionTimeStats[1],
                        mstBatteryConsumedStats[0], mstBatteryLeftStats[0],
                        mstMissionBatteryConsumedStats[0], mstMissionBatteryConsumedStats[1],
                        mstMissionBatteryLeftStats[0], mstMissionBatteryLeftStats[1],
                        mstDistanceStats[0], mstDistanceStats[1],
                        mstPointsStats[0],
                        mstAdaptStats[0],
                        mstDepotStats[0],
                        mstCostStats[0], mstCostStats[1],
                        mstCompTimeStats[0], mstCompTimeStats[1]);
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

    // Utility: Compute mean and std for List<Double>
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

    // Utility: Compute mean and std for List<Integer>
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

    /**
     * Write a CSV summary line. 
     *
     * Signature expects single double values for overall battery consumed/left,
     * but arrays for mission battery consumed/left (since we might want average + std dev).
     */
    private static void writeSummary(String csvFile,
                                     String scenario,
                                     String algorithm,
                                     int batteries,
                                     double payload,
                                     int serviceNodes,
                                     double[] energyStats,         // overall energy [mean, std]
                                     double[] timeStats,           // overall flight time [mean, std]
                                     double[] missionTimeStats,    // mission flight time [mean, std]
                                     double overallBatteryConsumed,// single double
                                     double overallBatteryLeft,    // single double
                                     double[] missionBatteryConsumedStats, // mission battery consumed [mean, std]
                                     double[] missionBatteryLeftStats,     // mission battery left [mean, std]
                                     double[] distanceStats,       // distance [mean, std]
                                     double[] pointsStats,         // points visited [mean, std]
                                     double[] adaptStats,          // adaptability [mean, std]
                                     double[] depotStats,          // depot returns [mean, std]
                                     double[] costStats,           // cost [mean, std]
                                     double[] compTimeStats)       // computation time [mean, std]
    {
        try (PrintWriter writer = new PrintWriter(new FileWriter(csvFile, true))) {
            // Write columns in the new order. 
            writer.printf("%s,%s,%d,%.1f,%d,%d," +
                            "%.2f,%.2f," +                // OverallEnergy mean, std
                            "%.2f,%.2f," +                // OverallTime mean, std
                            "%.2f,%.2f," +                // MissionTime mean, std
                            "%.2f,%.2f," +                // OverallBatteryConsumed, OverallBatteryLeft
                            "%.2f,%.2f,%.2f,%.2f," +      // MissionBatteryConsumedAvg, MissionBatteryConsumedStd, MissionBatteryLeftAvg, MissionBatteryLeftStd
                            "%.2f,%.2f,%.2f,%.2f,%.2f," + // Distance mean, std, Points, Adapt, Depot
                            "%.2f,%.2f,%.2f,%.2f%n",      // Cost mean, std, CompTime mean, std
                    scenario, algorithm, batteries, payload, serviceNodes, serviceNodes,
                    energyStats[0], energyStats[1],
                    timeStats[0], timeStats[1],
                    missionTimeStats[0], missionTimeStats[1],
                    overallBatteryConsumed, overallBatteryLeft,
                    missionBatteryConsumedStats[0], missionBatteryConsumedStats[1],
                    missionBatteryLeftStats[0], missionBatteryLeftStats[1],
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
