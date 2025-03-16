package drone;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/**
 * 3D route planning that supports three approaches:
 *   1) Basic Greedy (nearest neighbor based on distance)
 *   2) Energy-Based Greedy (minimizing energy cost with vertical movement)
 *   3) MST-based TSP Approximation
 */
public class Algorithms3D {

    // Constants updated with DJI FlyCart 30 specs
    private static final double ARC_PEAK = 50.0;          // Reasonable flight height (m)
    private static final int ARC_SAMPLES = 50;            // Number of arc points
    private static final double ONE_GALLON_KG = 3.785;    // kg per gallon
    private static final double WATER_PER_FIRE = ONE_GALLON_KG;  // Water per fire (kg)
    private static final double HOVER_FIRE = 30.0;        // seconds for fire hover
    private static final double HOVER_REFILL = 60.0;      // seconds for refill hover (updated to 1 minute)
    private static final double HOVER_BATTERY_REPLACE = 30.0; // seconds for battery replacement
    private static final double CRUISE_SPEED = 20.0;      // m/s (FlyCart 30 max speed)
    private static final double VERT_SPEED = 5.0;         // m/s vertical speed
    private static final double POWER_PER_KG = 200;       // W per kg
    private static final double DRONE_EMPTY = 10.0;       // kg (assumed drone weight)
    private static final double MAX_WATER_PAYLOAD = 20.0; // kg (within 30 kg dual battery payload limit)
    private static final double INITIAL_BATTERY = 3969.0; // Wh (FlyCart 30 battery)
    private static final double BATTERY_THRESHOLD = INITIAL_BATTERY * 0.05; // 5% threshold (~198.45 Wh)

    /**
     * Main entry for route planning.
     * methodChoice: 1 = Basic Greedy, 2 = Energy-Based Greedy, 3 = MST-based TSP Approximation.
     */
    public static List<SpatialNode> planRoute(
            int methodChoice,
            Graph3D graph,
            DroneNode depot,
            List<FireSite> fires,
            List<RendezvousPoint> rendezvous,
            double fullWaterKg
    ) {
        if (fullWaterKg > MAX_WATER_PAYLOAD) {
            System.out.println("Warning: Water payload exceeds FlyCart 30 max (20 kg). Adjusting to 20 kg.");
            fullWaterKg = MAX_WATER_PAYLOAD;
        }
        switch (methodChoice) {
            case 1:
                return basicGreedyRoute(depot, fires, rendezvous, fullWaterKg);
            case 2:
                return energyBasedRoute(graph, depot, fires, rendezvous, fullWaterKg);
            case 3:
                return mstTspApprox(graph, depot, fires, rendezvous, fullWaterKg);
            default:
                System.out.println("Invalid choice; using Basic Greedy as default.");
                return basicGreedyRoute(depot, fires, rendezvous, fullWaterKg);
        }
    }

    // ----------------- 1) Basic Greedy (Nearest Neighbor) -----------------
    private static List<SpatialNode> basicGreedyRoute(
            DroneNode depot,
            List<FireSite> fires,
            List<RendezvousPoint> rendezvous,
            double fullWaterKg
    ) {
        List<SpatialNode> route = new ArrayList<>();
        double water = fullWaterKg;
        double battery = INITIAL_BATTERY * 3600.0; // Convert Wh to Joules (1 Wh = 3600 J)
        SpatialNode current = new Waypoint(depot.getX(), depot.getY(), depot.getZ(), "DepotStart");
        route.add(current);
        List<FireSite> unvisited = new ArrayList<>(fires);

        while (!unvisited.isEmpty()) {
            // Estimate energy cost to the next fire site
            FireSite nextFire = null;
            double minDist = Double.MAX_VALUE;
            double minDistEnergy = Double.MAX_VALUE;
            for (FireSite fs : unvisited) {
                double dist = horizDist(current, fs);
                double energy = estimateEnergyCost(current, fs, water);
                if (dist < minDist) {
                    minDist = dist;
                    nextFire = fs;
                    minDistEnergy = energy;
                }
            }

            // Check if battery is sufficient for the next leg
            if (battery - minDistEnergy < BATTERY_THRESHOLD * 3600.0) {
                // Battery too low; return to depot for replacement
                route.addAll(createArc(current, depot, ARC_PEAK, ARC_SAMPLES));
                route.add(new Waypoint(depot.getX(), depot.getY(), depot.getZ(), "Hover30sBatteryReplace"));
                battery = INITIAL_BATTERY * 3600.0; // Reset battery
                current = depot;
            }

            // Check if water is sufficient
            if (water < WATER_PER_FIRE) {
                RendezvousPoint refill = findNearestRefill(current, rendezvous);
                double energyToRefill = estimateEnergyCost(current, refill, water);
                if (battery - energyToRefill < BATTERY_THRESHOLD * 3600.0) {
                    // Battery too low; return to depot first
                    route.addAll(createArc(current, depot, ARC_PEAK, ARC_SAMPLES));
                    route.add(new Waypoint(depot.getX(), depot.getY(), depot.getZ(), "Hover30sBatteryReplace"));
                    battery = INITIAL_BATTERY * 3600.0;
                    current = depot;
                }
                route.addAll(createArc(current, refill, ARC_PEAK, ARC_SAMPLES));
                route.add(new Waypoint(refill.getX(), refill.getY(), refill.getZ(), "Hover60sRefill"));
                battery -= energyToRefill + HOVER_REFILL * (DRONE_EMPTY + water) * POWER_PER_KG;
                water = fullWaterKg;
                current = refill;
            }

            // Travel to the fire site
            route.addAll(createArc(current, nextFire, ARC_PEAK, ARC_SAMPLES));
            route.add(new Waypoint(nextFire.getX(), nextFire.getY(), nextFire.getZ(), "Hover30sFire"));
            battery -= minDistEnergy + HOVER_FIRE * (DRONE_EMPTY + water) * POWER_PER_KG;
            water = Math.max(0, water - WATER_PER_FIRE);
            current = nextFire;
            unvisited.remove(nextFire);
        }

        // Final return to depot
        double energyToDepot = estimateEnergyCost(current, depot, water);
        if (battery - energyToDepot < BATTERY_THRESHOLD * 3600.0) {
            // Battery too low; replace before returning
            route.addAll(createArc(current, depot, ARC_PEAK, ARC_SAMPLES));
            route.add(new Waypoint(depot.getX(), depot.getY(), depot.getZ(), "Hover30sBatteryReplace"));
            battery = INITIAL_BATTERY * 3600.0;
            current = depot;
        }
        route.addAll(createArc(current, depot, ARC_PEAK, ARC_SAMPLES));
        return route;
    }

    // ----------------- 2) Energy-Based Greedy -----------------
    private static List<SpatialNode> energyBasedRoute(
            Graph3D graph,
            DroneNode depot,
            List<FireSite> fires,
            List<RendezvousPoint> rendezvous,
            double fullWaterKg
    ) {
        List<SpatialNode> route = new ArrayList<>();
        double water = fullWaterKg;
        double battery = INITIAL_BATTERY * 3600.0; // Convert Wh to Joules
        SpatialNode currentPos = new Waypoint(depot.getX(), depot.getY(), depot.getZ(), "DepotStart");
        route.add(currentPos);
        List<FireSite> unvisited = new ArrayList<>(fires);

        while (!unvisited.isEmpty()) {
            FireSite nextFire = null;
            double minEnergy = Double.MAX_VALUE;
            for (FireSite fs : unvisited) {
                double cost = estimateEnergyCost(currentPos, fs, water);
                if (cost < minEnergy) {
                    minEnergy = cost;
                    nextFire = fs;
                }
            }

            if (battery - minEnergy < BATTERY_THRESHOLD * 3600.0) {
                route.addAll(createArc(currentPos, depot, ARC_PEAK, ARC_SAMPLES));
                route.add(new Waypoint(depot.getX(), depot.getY(), depot.getZ(), "Hover30sBatteryReplace"));
                battery = INITIAL_BATTERY * 3600.0;
                currentPos = depot;
            }

            if (water < WATER_PER_FIRE) {
                RendezvousPoint refill = findNearestRefill(currentPos, rendezvous);
                double energyToRefill = estimateEnergyCost(currentPos, refill, water);
                if (battery - energyToRefill < BATTERY_THRESHOLD * 3600.0) {
                    route.addAll(createArc(currentPos, depot, ARC_PEAK, ARC_SAMPLES));
                    route.add(new Waypoint(depot.getX(), depot.getY(), depot.getZ(), "Hover30sBatteryReplace"));
                    battery = INITIAL_BATTERY * 3600.0;
                    currentPos = depot;
                }
                route.addAll(createArc(currentPos, refill, ARC_PEAK, ARC_SAMPLES));
                route.add(new Waypoint(refill.getX(), refill.getY(), refill.getZ(), "Hover60sRefill"));
                battery -= energyToRefill + HOVER_REFILL * (DRONE_EMPTY + water) * POWER_PER_KG;
                water = fullWaterKg;
                currentPos = refill;
            }

            route.addAll(createArc(currentPos, nextFire, ARC_PEAK, ARC_SAMPLES));
            route.add(new Waypoint(nextFire.getX(), nextFire.getY(), nextFire.getZ(), "Hover30sFire"));
            battery -= minEnergy + HOVER_FIRE * (DRONE_EMPTY + water) * POWER_PER_KG;
            water = Math.max(0, water - WATER_PER_FIRE);
            currentPos = nextFire;
            unvisited.remove(nextFire);
        }

        double energyToDepot = estimateEnergyCost(currentPos, depot, water);
        if (battery - energyToDepot < BATTERY_THRESHOLD * 3600.0) {
            route.addAll(createArc(currentPos, depot, ARC_PEAK, ARC_SAMPLES));
            route.add(new Waypoint(depot.getX(), depot.getY(), depot.getZ(), "Hover30sBatteryReplace"));
            battery = INITIAL_BATTERY * 3600.0;
            currentPos = depot;
        }
        route.addAll(createArc(currentPos, depot, ARC_PEAK, ARC_SAMPLES));
        return route;
    }

    private static double estimateEnergyCost(SpatialNode start, SpatialNode end, double water) {
        double dx = end.getX() - start.getX();
        double dy = end.getY() - start.getY();
        double dz = end.getZ() - start.getZ();
        double distHoriz = Math.sqrt(dx * dx + dz * dz);
        double distVert = Math.abs(dy);
        double tHoriz = distHoriz / CRUISE_SPEED;
        double tVert = distVert / VERT_SPEED;
        double weight = DRONE_EMPTY + water;
        return (tHoriz + tVert) * weight * POWER_PER_KG;
    }

    // ----------------- 3) MST-Based TSP Approximation -----------------
    private static List<SpatialNode> mstTspApprox(
            Graph3D graph,
            DroneNode depot,
            List<FireSite> fires,
            List<RendezvousPoint> rendezvous,
            double fullWaterKg
    ) {
        List<SpatialNode> allNodes = new ArrayList<>();
        allNodes.add(depot);
        allNodes.addAll(fires);
        allNodes.addAll(rendezvous);

        List<Edge> mstEdges = buildMST(allNodes);
        List<SpatialNode> nodeOrder = preOrderTraversal(depot, allNodes, mstEdges);

        List<SpatialNode> route = new ArrayList<>();
        double water = fullWaterKg;
        double battery = INITIAL_BATTERY * 3600.0;
        SpatialNode current = new Waypoint(depot.getX(), depot.getY(), depot.getZ(), "DepotStart");
        route.add(current);

        for (int i = 1; i < nodeOrder.size(); i++) {
            SpatialNode next = nodeOrder.get(i);
            double energyToNext = estimateEnergyCost(current, next, water);
            if (battery - energyToNext < BATTERY_THRESHOLD * 3600.0) {
                route.addAll(createArc(current, depot, ARC_PEAK, ARC_SAMPLES));
                route.add(new Waypoint(depot.getX(), depot.getY(), depot.getZ(), "Hover30sBatteryReplace"));
                battery = INITIAL_BATTERY * 3600.0;
                current = depot;
            }

            if (water < WATER_PER_FIRE) {
                RendezvousPoint refill = findNearestRefill(current, rendezvous);
                double energyToRefill = estimateEnergyCost(current, refill, water);
                if (battery - energyToRefill < BATTERY_THRESHOLD * 3600.0) {
                    route.addAll(createArc(current, depot, ARC_PEAK, ARC_SAMPLES));
                    route.add(new Waypoint(depot.getX(), depot.getY(), depot.getZ(), "Hover30sBatteryReplace"));
                    battery = INITIAL_BATTERY * 3600.0;
                    current = depot;
                }
                route.addAll(createArc(current, refill, ARC_PEAK, ARC_SAMPLES));
                route.add(new Waypoint(refill.getX(), refill.getY(), refill.getZ(), "Hover60sRefill"));
                battery -= energyToRefill + HOVER_REFILL * (DRONE_EMPTY + water) * POWER_PER_KG;
                water = fullWaterKg;
                current = refill;
            }

            route.addAll(createArc(current, next, ARC_PEAK, ARC_SAMPLES));
            battery -= energyToNext;
            if (next instanceof FireSite) {
                route.add(new Waypoint(next.getX(), next.getY(), next.getZ(), "Hover30sFire"));
                battery -= HOVER_FIRE * (DRONE_EMPTY + water) * POWER_PER_KG;
                water = Math.max(0, water - WATER_PER_FIRE);
            }
            current = next;
        }

        double energyToDepot = estimateEnergyCost(current, depot, water);
        if (battery - energyToDepot < BATTERY_THRESHOLD * 3600.0) {
            route.addAll(createArc(current, depot, ARC_PEAK, ARC_SAMPLES));
            route.add(new Waypoint(depot.getX(), depot.getY(), depot.getZ(), "Hover30sBatteryReplace"));
            battery = INITIAL_BATTERY * 3600.0;
            current = depot;
        }
        route.addAll(createArc(current, depot, ARC_PEAK, ARC_SAMPLES));
        return route;
    }

    private static List<Edge> buildMST(List<SpatialNode> nodes) {
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

    // ----------------- Common Helper Methods -----------------
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
            arcPoints.add(new Waypoint(x, y, z, "ArcSegment"));
        }
        return arcPoints;
    }

    private static RendezvousPoint findNearestRefill(SpatialNode current, List<RendezvousPoint> rvs) {
        return rvs.stream()
                .min(Comparator.comparingDouble(r -> horizDist(current, r)))
                .orElse(null);
    }

    private static double horizDist(SpatialNode a, SpatialNode b) {
        double dx = a.getX() - b.getX();
        double dz = a.getZ() - b.getZ();
        return Math.sqrt(dx * dx + dz * dz);
    }

    /**
     * Calculate energy consumption along the route.
     * Prints a summary at each key node (hover at fire/refill, battery replace, or depot).
     */
    public static double calculateEnergyForRoute(List<SpatialNode> route, double fullWaterKg) {
        double totalJoules = 0;
        double water = fullWaterKg;
        double totalDistance = 0;
        double cumulativeFlightTime = 0;
        double initialBattery = INITIAL_BATTERY; // Wh
        double currentBattery = initialBattery * 3600.0; // Joules
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
            double weight = DRONE_EMPTY + water;
            double legJoules = flightTime * weight * POWER_PER_KG;
            String label = (to instanceof Waypoint) ? ((Waypoint)to).getLabel() : "";
            if (label.contains("Hover30sFire")) {
                legJoules += HOVER_FIRE * weight * POWER_PER_KG;
                water = Math.max(0, water - WATER_PER_FIRE);
            } else if (label.contains("Hover60sRefill")) {
                legJoules += HOVER_REFILL * weight * POWER_PER_KG;
                water = fullWaterKg;
            } else if (label.contains("Hover30sBatteryReplace")) {
                legJoules += HOVER_BATTERY_REPLACE * weight * POWER_PER_KG;
                currentBattery = initialBattery * 3600.0; // Reset battery
            }
            totalJoules += legJoules;
            currentBattery -= legJoules;
            if (currentBattery < 0) {
                System.out.println("Warning: Battery went negative during simulation!");
                currentBattery = 0;
            }
            if (label.contains("Hover30sFire") || label.contains("Hover60sRefill") || label.contains("DepotStart") || label.contains("Hover30sBatteryReplace")) {
                double whConsumed = totalJoules / 3600.0;
                double batteryLeft = currentBattery / 3600.0;
                double waterUsed = fullWaterKg - water;
                System.out.printf("Node %2d (%s): Pos(%.1f, %.1f, %.1f) | Total Distance: %.1f m | Flight Time: %.1f s | Battery Consumed: %.2f Wh | Battery Left: %.2f Wh | Water Used: %.2f kg | Water Left: %.2f kg\n",
                        i, label, to.getX(), to.getY(), to.getZ(), totalDistance, cumulativeFlightTime, whConsumed, batteryLeft, waterUsed, water);
            }
        }
        return totalJoules;
    }
}