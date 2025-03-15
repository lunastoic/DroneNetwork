package drone;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.Scanner;

/**
 * Main simulation:
 *  1) Asks the user for simulation parameters: area width, length, altitude, # fires, and # rendezvous.
 *  2) The drone starts at the origin with 8 gallons of water (~30.28 kg).
 *  3) Fire sites and rendezvous (refill) points are placed randomly on the ground (y = 0).
 *  4) The route is planned using Algorithms3D.planRoute (which supports multiple approaches).
 *  5) The energy consumption is calculated and a mission summary is printed.
 *  6) A 3D visualization of the flight path is launched.
 */
public class DroneFireSuppressionSim {
    private static final double DRONE_BATTERY = 3969;           // Wh
    private static final double GALLONS = 8;                    // 8 gallons
    private static final double ONE_GALLON_KG = 3.785;          // kg per gallon
    private static final double FULL_WATER = GALLONS * ONE_GALLON_KG; // ~30.28 kg

    public static void main(String[] args) {
        Scanner sc = new Scanner(System.in);
        System.out.print("Enter simulation area width (m): ");
        double width = sc.nextDouble();
        System.out.print("Enter simulation area length (m): ");
        double length = sc.nextDouble();
        System.out.print("Enter simulation area maximum altitude (m): ");
        double altitude = sc.nextDouble();
        System.out.print("Enter number of fire sites: ");
        int numFires = sc.nextInt();
        System.out.print("Enter number of rendezvous points: ");
        int numRendezvous = sc.nextInt();
        System.out.print("Choose route planning approach (1=BasicGreedy, 2=EnergyBased, 3=MST, 4=Metaheuristic): ");
        int approachChoice = sc.nextInt();
        sc.close();

        // Build the 3D environment.
        Graph3D graph = new Graph3D(width, length, altitude, 1000);

        // Create the drone at origin.
        DroneNode drone = new DroneNode(-1, 0, 0, 0, DRONE_BATTERY, FULL_WATER);
        graph.addNode(drone);

        // Create fire sites (placed randomly on ground level, y = 0).
        List<FireSite> fires = new ArrayList<>();
        Random rand = new Random();
        for (int i = 0; i < numFires; i++) {
            double x = rand.nextDouble() * width;
            double z = rand.nextDouble() * length;
            FireSite fs = new FireSite(i, x, 0, z);
            fires.add(fs);
            graph.addNode(fs);
        }

        // Create rendezvous (refill) points (also on ground level).
        List<RendezvousPoint> rPoints = new ArrayList<>();
        for (int i = 0; i < numRendezvous; i++) {
            double x = rand.nextDouble() * width;
            double z = rand.nextDouble() * length;
            RendezvousPoint rp = new RendezvousPoint(numFires + i, x, 0, z);
            rPoints.add(rp);
            graph.addNode(rp);
        }

        // Plan the route using the chosen approach.
        List<SpatialNode> route = Algorithms3D.planRoute(approachChoice, graph, drone, fires, rPoints, FULL_WATER);

        // Compute energy consumption and print mission summary.
        double totalEnergyJ = Algorithms3D.calculateEnergyForRoute(route, FULL_WATER);
        System.out.printf("Total Energy Consumption: %.2f Wh\n", totalEnergyJ / 3600.0);

        // Launch the 3D visualization.
        DroneNetworkVisualization3D.launchVisualizer(graph, route);
    }
}


