package drone;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * 3D route planning for a single-agent drone in dynamic emergency scenarios.
 * Implements Greedy and MST-based TSP Approximation algorithms.
 */
public class Algorithms3D {

    // Constants based on DJI FlyCart 30 specs
    private static final int ARC_SAMPLES = 50;            // Number of arc points for smooth paths
    private static final double HOVER_TIME = 30.0;        // Seconds for hover at each service point
    private static final double CRUISE_SPEED = 20.0;      // m/s (max horizontal speed)
    private static final double VERT_SPEED = 5.0;         // m/s (max ascent/descent speed)
    private static final double POWER_PER_KG = 200;       // W per kg (assumed, as in original code)
    private static final double DRONE_WEIGHT = 65.0;      // kg (with batteries)
    private static final double INITIAL_BATTERY_2 = 3969.0; // Wh (dual battery mode)
    private static final double INITIAL_BATTERY_1 = 1984.5; // Wh (single battery mode)
    private static final double BATTERY_THRESHOLD_PERCENT = 0.05; // 5% threshold

    // Random number generator for dynamic weight updates
    private static final Random RAND = new Random();

    /**
     * Main entry for route planning.
     * methodChoice: 1 = Greedy, 3 = MST-based TSP Approximation.
     * maxAltitude: User-inputted cruising altitude (m).
     * numBatteries: 1 or 2, affects battery capacity.
     * payloadWeight: 0-30 kg, affects total weight.
     */
    public static List<SpatialNode> planRoute(
            int methodChoice,
            Graph3D graph,
            DroneNode depot,
            List<ServicePoint> servicePoints,
            double maxAltitude,
            int numBatteries,
            double payloadWeight
    ) {
        double initialBattery = (numBatteries == 1) ? INITIAL_BATTERY_1 : INITIAL_BATTERY_2;
        double totalWeight = DRONE_WEIGHT + payloadWeight;
        switch (methodChoice) {
            case 1:
                return greedyRoute(depot, servicePoints, maxAltitude, initialBattery, totalWeight);
            case 3:
                return mstTspApprox(graph, depot, servicePoints, maxAltitude, initialBattery, totalWeight);
            default:
                System.out.println("Invalid choice; using Greedy as default.");
                return greedyRoute(depot, servicePoints, maxAltitude, initialBattery, totalWeight);
        }
    }

    // ----------------- 1) Greedy Route -----------------
    private static List<SpatialNode> greedyRoute(
            DroneNode depot,
            List<ServicePoint> servicePoints,
            double maxAltitude,
            double initialBattery,
            double totalWeight
    ) {
        List<SpatialNode> route = new ArrayList<>();
        double battery = initialBattery * 3600.0; // Convert Wh to Joules
        double batteryThreshold = initialBattery * BATTERY_THRESHOLD_PERCENT * 3600.0;
        SpatialNode current = new Waypoint(depot.getX(), depot.getY(), depot.getZ(), "DepotStart");
        route.add(current);

        // Ascend to max altitude at the start
        route.addAll(createArc(current, new Waypoint(depot.getX(), maxAltitude, depot.getZ(), "Temp"), maxAltitude, ARC_SAMPLES));
        current = new Waypoint(depot.getX(), maxAltitude, depot.getZ(), "CruisingAltitude");

        // Initialize weights for service points (simulating priority/urgency)
        List<ServicePoint> unvisited = new ArrayList<>(servicePoints);
        double[] weights = new double[unvisited.size()];
        for (int i = 0; i < weights.length; i++) {
            weights[i] = RAND.nextDouble() * 9 + 1; // Random weight between 1 and 10
        }

        while (!unvisited.isEmpty()) {
            // Find the next service point based on distance and weight
            ServicePoint nextPoint = null;
            double minScore = Double.MAX_VALUE;
            double minEnergy = Double.MAX_VALUE;
            int nextIndex = -1;
            for (int i = 0; i < unvisited.size(); i++) {
                ServicePoint sp = unvisited.get(i);
                double dist = horizDist(current, sp);
                double energy = estimateEnergyCost(current, sp, totalWeight);
                double score = dist / weights[i]; // Higher weight reduces score, prioritizing high-priority points
                if (score < minScore) {
                    minScore = score;
                    minEnergy = energy;
                    nextPoint = sp;
                    nextIndex = i;
                }
            }

            // Check if battery is sufficient for the next leg
            if (battery - minEnergy < batteryThreshold) {
                route.addAll(createArc(current, depot, maxAltitude, ARC_SAMPLES));
                route.addAll(createArc(new Waypoint(depot.getX(), maxAltitude, depot.getZ(), "Temp"), new Waypoint(depot.getX(), 0, depot.getZ(), "Temp"), maxAltitude, ARC_SAMPLES));
                route.add(new Waypoint(depot.getX(), 0, depot.getZ(), "DepotBatteryReplaced"));
                battery = initialBattery * 3600.0; // Reset battery
                current = new Waypoint(depot.getX(), 0, depot.getZ(), "DepotBatteryReplaced");
                route.addAll(createArc(current, new Waypoint(depot.getX(), maxAltitude, depot.getZ(), "Temp"), maxAltitude, ARC_SAMPLES));
                current = new Waypoint(depot.getX(), maxAltitude, depot.getZ(), "CruisingAltitude");
            }

            // Travel to the service point at max altitude, then descend to 10m
            route.addAll(createArc(current, new Waypoint(nextPoint.getX(), maxAltitude, nextPoint.getZ(), "Temp"), maxAltitude, ARC_SAMPLES));
            current = new Waypoint(nextPoint.getX(), maxAltitude, nextPoint.getZ(), "AboveServicePoint");
            route.addAll(createArc(current, new Waypoint(nextPoint.getX(), 10.0, nextPoint.getZ(), "Temp"), maxAltitude, ARC_SAMPLES));
            route.add(new Waypoint(nextPoint.getX(), 10.0, nextPoint.getZ(), "Hover30sService", weights[nextIndex]));
            battery -= minEnergy + HOVER_TIME * totalWeight * POWER_PER_KG;
            current = new Waypoint(nextPoint.getX(), 10.0, nextPoint.getZ(), "ServiceCompleted");
            route.addAll(createArc(current, new Waypoint(nextPoint.getX(), maxAltitude, nextPoint.getZ(), "Temp"), maxAltitude, ARC_SAMPLES));
            current = new Waypoint(nextPoint.getX(), maxAltitude, nextPoint.getZ(), "CruisingAltitude");

            // Update weights dynamically
            unvisited.remove(nextPoint);
            weights[nextIndex] = 0; // Visited point's weight set to 0
            for (int i = 0; i < unvisited.size(); i++) {
                weights[i] = Math.min(10, weights[i] + RAND.nextDouble() * 2); // Increase by up to 2, cap at 10
            }
        }

        // Final return to depot
        double energyToDepot = estimateEnergyCost(current, depot, totalWeight);
        if (battery - energyToDepot < batteryThreshold) {
            route.addAll(createArc(current, depot, maxAltitude, ARC_SAMPLES));
            route.addAll(createArc(new Waypoint(depot.getX(), maxAltitude, depot.getZ(), "Temp"), new Waypoint(depot.getX(), 0, depot.getZ(), "Temp"), maxAltitude, ARC_SAMPLES));
            route.add(new Waypoint(depot.getX(), 0, depot.getZ(), "DepotBatteryReplaced"));
            battery = initialBattery * 3600.0;
            current = new Waypoint(depot.getX(), 0, depot.getZ(), "DepotBatteryReplaced");
            route.addAll(createArc(current, new Waypoint(depot.getX(), maxAltitude, depot.getZ(), "Temp"), maxAltitude, ARC_SAMPLES));
            current = new Waypoint(depot.getX(), maxAltitude, depot.getZ(), "CruisingAltitude");
        }
        route.addAll(createArc(current, depot, maxAltitude, ARC_SAMPLES));
        route.addAll(createArc(new Waypoint(depot.getX(), maxAltitude, depot.getZ(), "Temp"), new Waypoint(depot.getX(), 0, depot.getZ(), "Temp"), maxAltitude, ARC_SAMPLES));
        return route;
    }

    // ----------------- 3) MST-based TSP Approximation -----------------
    private static List<SpatialNode> mstTspApprox(
            Graph3D graph,
            DroneNode depot,
            List<ServicePoint> servicePoints,
            double maxAltitude,
            double initialBattery,
            double totalWeight
    ) {
        List<SpatialNode> route = new ArrayList<>();
        double battery = initialBattery * 3600.0;
        double batteryThreshold = initialBattery * BATTERY_THRESHOLD_PERCENT * 3600.0;
        SpatialNode current = new Waypoint(depot.getX(), depot.getY(), depot.getZ(), "DepotStart");
        route.add(current);

        // Ascend to max altitude at the start
        route.addAll(createArc(current, new Waypoint(depot.getX(), maxAltitude, depot.getZ(), "Temp"), maxAltitude, ARC_SAMPLES));
        current = new Waypoint(depot.getX(), maxAltitude, depot.getZ(), "CruisingAltitude");

        // Initialize weights for service points
        List<SpatialNode> unvisited = new ArrayList<>(servicePoints);
        double[] weights = new double[unvisited.size()];
        for (int i = 0; i < weights.length; i++) {
            weights[i] = RAND.nextDouble() * 9 + 1; // Random weight between 1 and 10
        }

        while (!unvisited.isEmpty()) {
            // Build MST with current weights
            List<SpatialNode> allNodes = new ArrayList<>();
            allNodes.add(depot);
            allNodes.addAll(unvisited);
            List<Edge> mstEdges = buildMST(allNodes, weights, unvisited);
            List<SpatialNode> nodeOrder = preOrderTraversal(depot, allNodes, mstEdges);

            for (int i = 1; i < nodeOrder.size(); i++) {
                SpatialNode next = nodeOrder.get(i);
                if (!(next instanceof ServicePoint)) continue;
                int index = unvisited.indexOf(next);
                if (index == -1) continue;

                double energyToNext = estimateEnergyCost(current, next, totalWeight);
                if (battery - energyToNext < batteryThreshold) {
                    route.addAll(createArc(current, depot, maxAltitude, ARC_SAMPLES));
                    route.addAll(createArc(new Waypoint(depot.getX(), maxAltitude, depot.getZ(), "Temp"), new Waypoint(depot.getX(), 0, depot.getZ(), "Temp"), maxAltitude, ARC_SAMPLES));
                    route.add(new Waypoint(depot.getX(), 0, depot.getZ(), "DepotBatteryReplaced"));
                    battery = initialBattery * 3600.0;
                    current = new Waypoint(depot.getX(), 0, depot.getZ(), "DepotBatteryReplaced");
                    route.addAll(createArc(current, new Waypoint(depot.getX(), maxAltitude, depot.getZ(), "Temp"), maxAltitude, ARC_SAMPLES));
                    current = new Waypoint(depot.getX(), maxAltitude, depot.getZ(), "CruisingAltitude");
                }

                route.addAll(createArc(current, new Waypoint(next.getX(), maxAltitude, next.getZ(), "Temp"), maxAltitude, ARC_SAMPLES));
                battery -= energyToNext;
                current = new Waypoint(next.getX(), maxAltitude, next.getZ(), "AboveServicePoint");
                route.addAll(createArc(current, new Waypoint(next.getX(), 10.0, next.getZ(), "Temp"), maxAltitude, ARC_SAMPLES));
                route.add(new Waypoint(next.getX(), 10.0, next.getZ(), "Hover30sService", weights[index]));
                battery -= HOVER_TIME * totalWeight * POWER_PER_KG;
                current = new Waypoint(next.getX(), 10.0, next.getZ(), "ServiceCompleted");
                route.addAll(createArc(current, new Waypoint(next.getX(), maxAltitude, next.getZ(), "Temp"), maxAltitude, ARC_SAMPLES));
                current = new Waypoint(next.getX(), maxAltitude, next.getZ(), "CruisingAltitude");

                // Update weights dynamically
                unvisited.remove(next);
                weights[index] = 0;
                for (int j = 0; j < unvisited.size(); j++) {
                    weights[j] = Math.min(10, weights[j] + RAND.nextDouble() * 2);
                }
            }
        }

        double energyToDepot = estimateEnergyCost(current, depot, totalWeight);
        if (battery - energyToDepot < batteryThreshold) {
            route.addAll(createArc(current, depot, maxAltitude, ARC_SAMPLES));
            route.addAll(createArc(new Waypoint(depot.getX(), maxAltitude, depot.getZ(), "Temp"), new Waypoint(depot.getX(), 0, depot.getZ(), "Temp"), maxAltitude, ARC_SAMPLES));
            route.add(new Waypoint(depot.getX(), 0, depot.getZ(), "DepotBatteryReplaced"));
            battery = initialBattery * 3600.0;
            current = new Waypoint(depot.getX(), 0, depot.getZ(), "DepotBatteryReplaced");
            route.addAll(createArc(current, new Waypoint(depot.getX(), maxAltitude, depot.getZ(), "Temp"), maxAltitude, ARC_SAMPLES));
            current = new Waypoint(depot.getX(), maxAltitude, depot.getZ(), "CruisingAltitude");
        }
        route.addAll(createArc(current, depot, maxAltitude, ARC_SAMPLES));
        route.addAll(createArc(new Waypoint(depot.getX(), maxAltitude, depot.getZ(), "Temp"), new Waypoint(depot.getX(), 0, depot.getZ(), "Temp"), maxAltitude, ARC_SAMPLES));
        return route;
    }

    private static double estimateEnergyCost(SpatialNode start, SpatialNode end, double totalWeight) {
        double dx = end.getX() - start.getX();
        double dy = end.getY() - start.getY();
        double dz = end.getZ() - start.getZ();
        double distHoriz = Math.sqrt(dx * dx + dz * dz);
        double distVert = Math.abs(dy);
        double tHoriz = distHoriz / CRUISE_SPEED;
        double tVert = distVert / VERT_SPEED;
        return (tHoriz + tVert) * totalWeight * POWER_PER_KG;
    }

    private static List<SpatialNode> createArc(SpatialNode start, SpatialNode end, double arcPeak, int numSamples) {
        List<SpatialNode> arcPoints = new ArrayList<>();
        double x1 = start.getX(), y1 = start.getY(), z1 = start.getZ();
        double x2 = end.getX(), y2 = end.getY(), z2 = end.getZ();
        for (int i = 1; i <= numSamples; i++) {
            double t = (double) i / numSamples;
            double x = x1 + t * (x2 - x1);
            double z = z1 + t * (z2 - z1);
            double baseY = (1 - t) * y1 + t * y2;
            double offsetY = -4 * arcPeak * t * (1 - t);
            double y = baseY + offsetY;
            if (y < 0) y = 0; // Ensure Y doesn't go below grid
            arcPoints.add(new Waypoint(x, y, z, "ArcSegment"));
        }
        return arcPoints;
    }

    private static double horizDist(SpatialNode a, SpatialNode b) {
        double dx = a.getX() - b.getX();
        double dz = a.getZ() - b.getZ();
        return Math.sqrt(dx * dx + dz * dz);
    }

    private static List<Edge> buildMST(List<SpatialNode> nodes, double[] weights, List<SpatialNode> unvisited) {
        List<Edge> mst = new ArrayList<>();
        if (nodes.isEmpty()) return mst;
        SpatialNode start = nodes.get(0);
        List<SpatialNode> visited = new ArrayList<>();
        visited.add(start);
        List<Edge> edges = new ArrayList<>();

        while (visited.size() < nodes.size()) {
            for (SpatialNode v : visited) {
                for (SpatialNode u : nodes) {
                    if (!visited.contains(u)) {
                        double cost = estimateSimpleDist(v, u);
                        int index = unvisited.indexOf(u);
                        if (index != -1) {
                            cost /= weights[index]; // Higher weight reduces cost
                        }
                        edges.add(new Edge(v, u, cost));
                    }
                }
            }
            Edge bestEdge = null;
            double bestCost = Double.MAX_VALUE;
            for (Edge e : edges) {
                if (visited.contains(e.a) && !visited.contains(e.b) && e.cost < bestCost) {
                    bestEdge = e;
                    bestCost = e.cost;
                }
            }
            if (bestEdge == null) break;
            mst.add(bestEdge);
            visited.add(bestEdge.b);
            edges.remove(bestEdge);
        }
        return mst;
    }

    private static double estimateSimpleDist(SpatialNode a, SpatialNode b) {
        double dx = a.getX() - b.getX();
        double dz = a.getZ() - b.getZ();
        return Math.sqrt(dx * dx + dz * dz);
    }

    private static List<SpatialNode> preOrderTraversal(SpatialNode start, List<SpatialNode> nodes, List<Edge> mst) {
        List<SpatialNode> result = new ArrayList<>();
        result.add(start);
        java.util.Map<SpatialNode, List<SpatialNode>> adj = new java.util.HashMap<>();
        for (SpatialNode n : nodes) {
            adj.put(n, new ArrayList<>());
        }
        for (Edge e : mst) {
            adj.get(e.a).add(e.b);
            adj.get(e.b).add(e.a);
        }
        java.util.Set<SpatialNode> visited = new java.util.HashSet<>();
        dfsPreOrder(start, adj, visited, result);
        return result;
    }

    private static void dfsPreOrder(SpatialNode current,
                                    java.util.Map<SpatialNode, List<SpatialNode>> adj,
                                    java.util.Set<SpatialNode> visited,
                                    List<SpatialNode> result) {
        visited.add(current);
        for (SpatialNode nxt : adj.get(current)) {
            if (!visited.contains(nxt)) {
                result.add(nxt);
                dfsPreOrder(nxt, adj, visited, result);
            }
        }
    }

    private static class Edge {
        SpatialNode a, b;
        double cost;
        Edge(SpatialNode a, SpatialNode b, double cost) {
            this.a = a;
            this.b = b;
            this.cost = cost;
        }
    }

    /**
     * Calculate metrics for the route.
     * Returns [energy (Wh), time (s), points visited, adaptability (%)]
     */
    public static double[] calculateMetricsForRoute(List<SpatialNode> route, int totalPoints) {
        double totalJoules = 0;
        double totalDistance = 0;
        double cumulativeFlightTime = 0;
        int pointsVisited = 0;
        List<Double> visitWeights = new ArrayList<>();

        System.out.println("\n--- Drone Mission Summary ---");
        for (int i = 0; i < route.size() - 1; i++) {
            SpatialNode from = route.get(i);
            SpatialNode to = route.get(i + 1);
            double dx = to.getX() - from.getX();
            double dy = to.getY() - from.getY();
            double dz = to.getZ() - from.getZ();
            double dist3D = Math.sqrt(dx * dx + dy * dy + dz * dz);
            totalDistance += dist3D;
            double tHoriz = 0;
            double horiz = Math.sqrt(dx * dx + dz * dz);
            if (horiz > 0.01) {
                tHoriz = horiz / CRUISE_SPEED;
            }
            double tVert = 0;
            if (Math.abs(dy) > 0.01) {
                tVert = Math.abs(dy) / VERT_SPEED;
            }
            double flightTime = tHoriz + tVert;
            cumulativeFlightTime += flightTime;
            double totalWeight = DRONE_WEIGHT + (to instanceof Waypoint ? ((Waypoint)to).getPayloadWeight() : 0);
            double legJoules = flightTime * totalWeight * POWER_PER_KG;
            String label = (to instanceof Waypoint) ? ((Waypoint)to).getLabel() : "";
            if (label.contains("Hover30sService")) {
                legJoules += HOVER_TIME * totalWeight * POWER_PER_KG;
                pointsVisited++;
                if (to instanceof Waypoint) {
                    double weight = ((Waypoint)to).getWeight();
                    visitWeights.add(weight);
                }
            }
            totalJoules += legJoules;
            if (label.contains("Hover30sService") || label.contains("DepotStart") || label.contains("DepotBatteryReplaced")) {
                double whConsumed = totalJoules / 3600.0;
                System.out.printf("Node %2d (%s): Pos(%.1f, %.1f, %.1f) | Total Distance: %.1f m | Flight Time: %.1f s | Energy Consumed: %.2f Wh\n",
                        i, label, to.getX(), to.getY(), to.getZ(), totalDistance, cumulativeFlightTime, whConsumed);
            }
        }

        // Calculate adaptability: % of high-priority points (weight >= 7) visited in first 50% of visits
        double adaptability = 0;
        if (pointsVisited > 0) {
            int firstHalf = (int) Math.ceil(pointsVisited / 2.0);
            int highPriorityCount = 0;
            for (int i = 0; i < Math.min(firstHalf, visitWeights.size()); i++) {
                if (visitWeights.get(i) >= 7) {
                    highPriorityCount++;
                }
            }
            adaptability = (double) highPriorityCount / firstHalf * 100;
        }

        return new double[]{totalJoules / 3600.0, cumulativeFlightTime, pointsVisited, adaptability};
    }
}