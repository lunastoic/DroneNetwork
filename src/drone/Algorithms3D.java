package drone;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/**
 * 3D route planning that supports multiple approaches:
 *   1) Basic Greedy (similar to FlightPlanner.planRoute)
 *   2) Energy-Based Greedy
 *   3) MST-based TSP Approximation
 *   4) Metaheuristic (placeholder; currently falls back to energy-based)
 */
public class Algorithms3D {

    // Common constants
    private static final double ARC_PEAK = 50.0;
    private static final int ARC_SAMPLES = 50;
    private static final double ONE_GALLON_KG = 3.785;
    private static final double WATER_PER_FIRE = ONE_GALLON_KG;
    private static final double HOVER_FIRE = 30.0;    // seconds for fire hover
    private static final double HOVER_REFILL = 60.0;  // seconds for refill hover
    private static final double CRUISE_SPEED = 15.0;  // m/s horizontal
    private static final double VERT_SPEED = 5.0;     // m/s vertical
    private static final double POWER_PER_KG = 200;   // W per kg
    private static final double DRONE_EMPTY = 10.0;   // kg

    /**
     * Main entry for route planning.
     * methodChoice: 1 = Basic Greedy, 2 = Energy-Based, 3 = MST, 4 = Metaheuristic.
     */
    public static List<SpatialNode> planRoute(
            int methodChoice,
            Graph3D graph,
            DroneNode depot,
            List<FireSite> fires,
            List<RendezvousPoint> rendezvous,
            double fullWaterKg
    ) {
        switch (methodChoice) {
            case 1:
                return basicGreedyRoute(depot, fires, rendezvous, fullWaterKg);
            case 2:
                return energyBasedRoute(graph, depot, fires, rendezvous, fullWaterKg);
            case 3:
                return mstTspApprox(graph, depot, fires, rendezvous, fullWaterKg);
            case 4:
            default:
                System.out.println("Metaheuristic approach not implemented; using Energy-Based approach.");
                return energyBasedRoute(graph, depot, fires, rendezvous, fullWaterKg);
        }
    }

    // ----------------- 1) Basic Greedy -----------------
    private static List<SpatialNode> basicGreedyRoute(
            DroneNode depot,
            List<FireSite> fires,
            List<RendezvousPoint> rendezvous,
            double fullWaterKg
    ) {
        List<SpatialNode> route = new ArrayList<>();
        double water = fullWaterKg;
        SpatialNode current = new Waypoint(depot.getX(), depot.getY(), depot.getZ(), "DepotStart");
        route.add(current);

        // Sort fires by X coordinate (simple ordering)
        List<FireSite> sortedFires = new ArrayList<>(fires);
        sortedFires.sort(Comparator.comparingDouble(FireSite::getX));

        for (FireSite fire : sortedFires) {
            if (water < WATER_PER_FIRE) {
                RendezvousPoint refill = findNearestRefill(current, rendezvous);
                route.addAll(createArc(current, refill, ARC_PEAK, ARC_SAMPLES));
                route.add(new Waypoint(refill.getX(), refill.getY(), refill.getZ(), "Hover30sRefill"));
                water = fullWaterKg;
                current = refill;
            }
            route.addAll(createArc(current, fire, ARC_PEAK, ARC_SAMPLES));
            route.add(new Waypoint(fire.getX(), fire.getY(), fire.getZ(), "Hover30sFire"));
            water = Math.max(0, water - WATER_PER_FIRE);
            current = fire;
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
            if (water < WATER_PER_FIRE) {
                RendezvousPoint refill = findNearestRefill(currentPos, rendezvous);
                route.addAll(createArc(currentPos, refill, ARC_PEAK, ARC_SAMPLES));
                route.add(new Waypoint(refill.getX(), refill.getY(), refill.getZ(), "Hover30sRefill"));
                water = fullWaterKg;
                currentPos = refill;
            }
            route.addAll(createArc(currentPos, nextFire, ARC_PEAK, ARC_SAMPLES));
            route.add(new Waypoint(nextFire.getX(), nextFire.getY(), nextFire.getZ(), "Hover30sFire"));
            water = Math.max(0, water - WATER_PER_FIRE);
            currentPos = nextFire;
            unvisited.remove(nextFire);
        }
        route.addAll(createArc(currentPos, depot, ARC_PEAK, ARC_SAMPLES));
        return route;
    }

    private static double estimateEnergyCost(SpatialNode start, SpatialNode end, double water) {
        double dx = end.getX() - start.getX();
        double dz = end.getZ() - start.getZ();
        double distHoriz = Math.sqrt(dx * dx + dz * dz);
        double tFlight = distHoriz / CRUISE_SPEED;
        double weight = DRONE_EMPTY + water;
        return tFlight * weight * POWER_PER_KG;
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
        SpatialNode current = new Waypoint(depot.getX(), depot.getY(), depot.getZ(), "DepotStart");
        route.add(current);

        for (int i = 1; i < nodeOrder.size(); i++) {
            SpatialNode next = nodeOrder.get(i);
            if (water < WATER_PER_FIRE) {
                RendezvousPoint refill = findNearestRefill(current, rendezvous);
                route.addAll(createArc(current, refill, ARC_PEAK, ARC_SAMPLES));
                route.add(new Waypoint(refill.getX(), refill.getY(), refill.getZ(), "Hover30sRefill"));
                water = fullWaterKg;
                current = refill;
            }
            route.addAll(createArc(current, next, ARC_PEAK, ARC_SAMPLES));
            if (next instanceof FireSite) {
                route.add(new Waypoint(next.getX(), next.getY(), next.getZ(), "Hover30sFire"));
                water = Math.max(0, water - WATER_PER_FIRE);
            }
            current = next;
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

    // ----------------- 4) Metaheuristic Placeholder -----------------
    private static List<SpatialNode> metaheuristicPlaceholder(
            Graph3D graph,
            DroneNode depot,
            List<FireSite> fires,
            List<RendezvousPoint> rendezvous,
            double fullWaterKg
    ) {
        System.out.println("Metaheuristic approach not implemented; using Energy-Based approach.");
        return energyBasedRoute(graph, depot, fires, rendezvous, fullWaterKg);
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
     * Instead of printing every leg's detail, this method prints a summary at each key node
     * (i.e. when hovering at a fire or refill, or at the depot).
     */
    public static double calculateEnergyForRoute(List<SpatialNode> route, double fullWaterKg) {
        double totalJoules = 0;
        double water = fullWaterKg;
        double totalDistance = 0;
        double cumulativeFlightTime = 0;
        double initialBattery = 3969.0; // Wh
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
            } else if (label.contains("Hover30sRefill")) {
                legJoules += HOVER_REFILL * weight * POWER_PER_KG;
                water = fullWaterKg;
            }
            totalJoules += legJoules;
            // Print summary only for key nodes
            if (label.contains("Hover30sFire") || label.contains("Hover30sRefill") || label.contains("DepotStart")) {
                double whConsumed = totalJoules / 3600.0;
                double batteryLeft = initialBattery - whConsumed;
                double waterUsed = fullWaterKg - water;
                System.out.printf("Node %2d (%s): Pos(%.1f, %.1f, %.1f) | Total Distance: %.1f m | Flight Time: %.1f s | Battery Consumed: %.2f Wh | Battery Left: %.2f Wh | Water Used: %.2f kg | Water Left: %.2f kg\n",
                        i, label, to.getX(), to.getY(), to.getZ(), totalDistance, cumulativeFlightTime, whConsumed, batteryLeft, waterUsed, water);
            }
        }
        return totalJoules;
    }
}



