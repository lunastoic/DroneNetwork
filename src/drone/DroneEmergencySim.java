package drone;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.Scanner;

public class DroneEmergencySim {

    private static final Random RAND = new Random();

    public static void main(String[] args) {
        // Get user inputs
        Scanner scanner = new Scanner(System.in);

        System.out.print("Enter simulation width (m, e.g., 500): ");
        double width = scanner.nextDouble();

        System.out.print("Enter simulation length (m, e.g., 500): ");
        double length = scanner.nextDouble();

        System.out.print("Enter cruising altitude (m, 50-400, e.g., 140): ");
        double maxAltitude = scanner.nextDouble();
        if (maxAltitude < 50) {
            System.out.println("Altitude too low; setting to 50m.");
            maxAltitude = 50;
        } else if (maxAltitude > 400) {
            System.out.println("Altitude too high for practical scenarios; setting to 400m.");
            maxAltitude = 400;
        }

        System.out.print("Enter number of service points (e.g., 5): ");
        int numServicePoints = scanner.nextInt();

        System.out.print("Enter number of batteries (1 or 2): ");
        int numBatteries = scanner.nextInt();
        if (numBatteries != 1 && numBatteries != 2) {
            System.out.println("Invalid input; defaulting to 2 batteries.");
            numBatteries = 2;
        }

        System.out.print("Enter payload weight (kg, 0-30, e.g., 20): ");
        double payloadWeight = scanner.nextDouble();
        double totalWeight = 65.0 + payloadWeight; // 65 kg base weight with batteries
        if (totalWeight > 95.0) {
            System.out.println("Total weight exceeds max takeoff weight (95 kg); capping payload at 30 kg.");
            payloadWeight = 30.0;
        }

        System.out.print("Enter number of trials per scenario (e.g., 10): ");
        int numTrials = scanner.nextInt();

        scanner.close();

        // Create CSV file with timestamp
        String timestamp = LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss"));
        String csvFile = "results_" + timestamp + ".csv";
        try (PrintWriter writer = new PrintWriter(new FileWriter(csvFile, true))) {
            // Write CSV headers
            writer.println("Trial,Algorithm,Batteries,Payload,Points,Energy,Time,PointsVisited,Adaptability");
        } catch (IOException e) {
            System.out.println("Error creating CSV file: " + e.getMessage());
            return;
        }

        // Run simulations for the scenario
        System.out.println("\nRunning simulations for " + numBatteries + " batteries, " + payloadWeight + " kg payload, " + numServicePoints + " points...");
        List<Double> greedyEnergies = new ArrayList<>();
        List<Double> greedyTimes = new ArrayList<>();
        List<Integer> greedyPointsVisited = new ArrayList<>();
        List<Double> greedyAdaptabilities = new ArrayList<>();
        List<Double> mstTspEnergies = new ArrayList<>();
        List<Double> mstTspTimes = new ArrayList<>();
        List<Integer> mstTspPointsVisited = new ArrayList<>();
        List<Double> mstTspAdaptabilities = new ArrayList<>();

        for (int trial = 1; trial <= numTrials; trial++) {
            // Create graph and nodes
            Graph3D graph = new Graph3D(width, maxAltitude, length);
            DroneNode depot = new DroneNode(0, 0, 0);
            graph.addNode(depot);

            List<ServicePoint> servicePoints = new ArrayList<>();
            for (int i = 0; i < numServicePoints; i++) {
                double x = RAND.nextDouble() * width;
                double z = RAND.nextDouble() * length;
                ServicePoint sp = new ServicePoint(x, 0, z);
                graph.addNode(sp);
                servicePoints.add(sp);
            }

            // Run Greedy algorithm
            List<SpatialNode> greedyRoute = Algorithms3D.planRoute(1, graph, depot, servicePoints, maxAltitude, numBatteries, payloadWeight);
            double[] greedyMetrics = Algorithms3D.calculateMetricsForRoute(greedyRoute, numServicePoints);
            double greedyEnergy = greedyMetrics[0];
            double greedyTime = greedyMetrics[1];
            int greedyPoints = (int) greedyMetrics[2];
            double greedyAdaptability = greedyMetrics[3];

            // Run MST-TSP algorithm
            List<SpatialNode> mstTspRoute = Algorithms3D.planRoute(3, graph, depot, servicePoints, maxAltitude, numBatteries, payloadWeight);
            double[] mstTspMetrics = Algorithms3D.calculateMetricsForRoute(mstTspRoute, numServicePoints);
            double mstTspEnergy = mstTspMetrics[0];
            double mstTspTime = mstTspMetrics[1];
            int mstTspPoints = (int) mstTspMetrics[2];
            double mstTspAdaptability = mstTspMetrics[3];

            // Store metrics for summary
            greedyEnergies.add(greedyEnergy);
            greedyTimes.add(greedyTime);
            greedyPointsVisited.add(greedyPoints);
            greedyAdaptabilities.add(greedyAdaptability);
            mstTspEnergies.add(mstTspEnergy);
            mstTspTimes.add(mstTspTime);
            mstTspPointsVisited.add(mstTspPoints);
            mstTspAdaptabilities.add(mstTspAdaptability);

            // Per-run console output
            System.out.println("\nTrial " + trial + ", Greedy:");
            System.out.printf("Energy: %.2f Wh\n", greedyEnergy);
            System.out.printf("Time: %.2f s\n", greedyTime);
            System.out.printf("Points Visited: %d/%d\n", greedyPoints, numServicePoints);
            System.out.printf("Adaptability: %.2f%%\n", greedyAdaptability);

            System.out.println("Trial " + trial + ", MST-TSP:");
            System.out.printf("Energy: %.2f Wh\n", mstTspEnergy);
            System.out.printf("Time: %.2f s\n", mstTspTime);
            System.out.printf("Points Visited: %d/%d\n", mstTspPoints, numServicePoints);
            System.out.printf("Adaptability: %.2f%%\n", mstTspAdaptability);

            // Per-run CSV output
            try (PrintWriter writer = new PrintWriter(new FileWriter(csvFile, true))) {
                writer.printf("%d,Greedy,%d,%.2f,%d,%.2f,%.2f,%d,%.2f\n",
                        trial, numBatteries, payloadWeight, numServicePoints, greedyEnergy, greedyTime, greedyPoints, greedyAdaptability);
                writer.printf("%d,MST-TSP,%d,%.2f,%d,%.2f,%.2f,%d,%.2f\n",
                        trial, numBatteries, payloadWeight, numServicePoints, mstTspEnergy, mstTspTime, mstTspPoints, mstTspAdaptability);
            } catch (IOException e) {
                System.out.println("Error writing to CSV file: " + e.getMessage());
            }

            // Visualize the last trial (optional, can comment out to speed up)
            if (trial == numTrials) {
                System.out.println("\nVisualizing Greedy route for trial " + trial + "...");
                DroneNetworkVisualization3D.launchVisualizer(graph, greedyRoute);
                System.out.println("Visualizing MST-TSP route for trial " + trial + "...");
                DroneNetworkVisualization3D.launchVisualizer(graph, mstTspRoute);
            }
        }

        // Compute summary statistics
        double[] greedyEnergyStats = computeStats(greedyEnergies.stream().mapToDouble(Double::doubleValue).toArray());
        double[] greedyTimeStats = computeStats(greedyTimes.stream().mapToDouble(Double::doubleValue).toArray());
        double[] greedyPointsStats = computeStats(greedyPointsVisited.stream().mapToDouble(Integer::doubleValue).toArray());
        double[] greedyAdaptabilityStats = computeStats(greedyAdaptabilities.stream().mapToDouble(Double::doubleValue).toArray());
        double[] mstTspEnergyStats = computeStats(mstTspEnergies.stream().mapToDouble(Double::doubleValue).toArray());
        double[] mstTspTimeStats = computeStats(mstTspTimes.stream().mapToDouble(Double::doubleValue).toArray());
        double[] mstTspPointsStats = computeStats(mstTspPointsVisited.stream().mapToDouble(Integer::doubleValue).toArray());
        double[] mstTspAdaptabilityStats = computeStats(mstTspAdaptabilities.stream().mapToDouble(Double::doubleValue).toArray());

        // Summary console output
        System.out.println("\nSummary for " + numBatteries + " batteries, " + payloadWeight + " kg payload, " + numServicePoints + " points (" + numTrials + " trials):");
        System.out.println("Greedy:");
        System.out.printf("  Energy: %.2f ± %.2f Wh\n", greedyEnergyStats[0], greedyEnergyStats[1]);
        System.out.printf("  Time: %.2f ± %.2f s\n", greedyTimeStats[0], greedyTimeStats[1]);
        System.out.printf("  Points Visited: %.1f/%d\n", greedyPointsStats[0], numServicePoints);
        System.out.printf("  Adaptability: %.2f%% ± %.2f%%\n", greedyAdaptabilityStats[0], greedyAdaptabilityStats[1]);
        System.out.println("MST-TSP:");
        System.out.printf("  Energy: %.2f ± %.2f Wh\n", mstTspEnergyStats[0], mstTspEnergyStats[1]);
        System.out.printf("  Time: %.2f ± %.2f s\n", mstTspTimeStats[0], mstTspTimeStats[1]);
        System.out.printf("  Points Visited: %.1f/%d\n", mstTspPointsStats[0], numServicePoints);
        System.out.printf("  Adaptability: %.2f%% ± %.2f%%\n", mstTspAdaptabilityStats[0], mstTspAdaptabilityStats[1]);

        // Summary CSV output
        try (PrintWriter writer = new PrintWriter(new FileWriter(csvFile, true))) {
            writer.println("\nScenario,Batteries,Payload,Points,Algorithm,EnergyAvg,EnergyStd,TimeAvg,TimeStd,PointsVisitedAvg,AdaptabilityAvg,AdaptabilityStd");
            writer.printf("%db_%.0fkg_%dp,%d,%.2f,%d,Greedy,%.2f,%.2f,%.2f,%.2f,%.1f,%.2f,%.2f\n",
                    numBatteries, payloadWeight, numServicePoints, numBatteries, payloadWeight, numServicePoints,
                    greedyEnergyStats[0], greedyEnergyStats[1], greedyTimeStats[0], greedyTimeStats[1],
                    greedyPointsStats[0], greedyAdaptabilityStats[0], greedyAdaptabilityStats[1]);
            writer.printf("%db_%.0fkg_%dp,%d,%.2f,%d,MST-TSP,%.2f,%.2f,%.2f,%.2f,%.1f,%.2f,%.2f\n",
                    numBatteries, payloadWeight, numServicePoints, numBatteries, payloadWeight, numServicePoints,
                    mstTspEnergyStats[0], mstTspEnergyStats[1], mstTspTimeStats[0], mstTspTimeStats[1],
                    mstTspPointsStats[0], mstTspAdaptabilityStats[0], mstTspAdaptabilityStats[1]);
            System.out.println("\nResults saved to " + csvFile);
        } catch (IOException e) {
            System.out.println("Error writing summary to CSV file: " + e.getMessage());
        }
    }

    private static double[] computeStats(double[] values) {
        double sum = 0;
        for (double v : values) {
            sum += v;
        }
        double mean = sum / values.length;
        double sumSquaredDiff = 0;
        for (double v : values) {
            double diff = v - mean;
            sumSquaredDiff += diff * diff;
        }
        double stdDev = Math.sqrt(sumSquaredDiff / values.length);
        return new double[]{mean, stdDev};
    }
}