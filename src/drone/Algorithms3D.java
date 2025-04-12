package drone;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * Implements Greedy (methodChoice = 1) and MST-based TSP Approximation (methodChoice = 3).
 * 
 * Key updates:
 * - Battery threshold is set to 15%.
 * - For any return to the depot (mid-route or final), a strict 4-step safe approach is enforced.
 * - In createBezierArc, if neither endpoint indicates a legitimate descent and the segment is horizontal,
 *   the altitude is forced constant (at least 50 m).
 */
public class Algorithms3D {

    private static final int ARC_SAMPLES = 50;
    private static final double HOVER_TIME = 30.0;
    private static final double CRUISE_SPEED = 20.0;
    private static final double VERT_SPEED = 5.0;
    private static final double POWER_PER_KG = 200;
    private static final double DRONE_WEIGHT = 65.0;
    private static final double INITIAL_BATTERY_2 = 3969.0;
    private static final double INITIAL_BATTERY_1 = 1984.5;
    private static final double BATTERY_THRESHOLD_PERCENT = 0.15;
    private static final double CURVATURE_FACTOR = 0.2;

    private static final Random RAND = new Random();

    /**
     * Main entry for route planning.
     */
    public static List<SpatialNode> planRoute(int methodChoice,
                                              Graph3D graph,
                                              DroneNode depot,
                                              List<ServicePoint> servicePoints,
                                              double maxAltitude,
                                              int numBatteries,
                                              double payloadWeight) {
        double totalWeight = DRONE_WEIGHT + payloadWeight;
        if (maxAltitude < 0 || maxAltitude > 6000) {
            System.out.println("Error: Max altitude must be between 0 and 6000m.");
            maxAltitude = Math.max(0, Math.min(6000, maxAltitude));
        }
        double initialBattery = (numBatteries == 1) ? INITIAL_BATTERY_1 : INITIAL_BATTERY_2;
        switch (methodChoice) {
            case 1:
                return greedyRoute(depot, servicePoints, maxAltitude, initialBattery, totalWeight);
            case 3:
                return mstTspApprox(graph, depot, servicePoints, maxAltitude, initialBattery, totalWeight);
            default:
                System.out.println("Invalid choice; defaulting to Greedy.");
                return greedyRoute(depot, servicePoints, maxAltitude, initialBattery, totalWeight);
        }
    }

    // ---------------- GREEDY ROUTE ----------------
    private static List<SpatialNode> greedyRoute(DroneNode depot,
                                                   List<ServicePoint> servicePoints,
                                                   double maxAltitude,
                                                   double initialBattery,
                                                   double totalWeight) {
        List<SpatialNode> route = new ArrayList<>();
        double battery = initialBattery * 3600.0; // Joules
        double batteryThreshold = initialBattery * BATTERY_THRESHOLD_PERCENT * 3600.0;

        SpatialNode current = new Waypoint(depot.getX(), depot.getY(), depot.getZ(), "DepotStart");
        route.add(current);

        // Ascend to cruising altitude.
        route.addAll(createBezierArc(current,
                new Waypoint(depot.getX(), maxAltitude, depot.getZ(), "AscendToCruise"),
                ARC_SAMPLES));
        current = new Waypoint(depot.getX(), maxAltitude, depot.getZ(), "CruisingAltitude");

        List<ServicePoint> unvisited = new ArrayList<>(servicePoints);
        double[] weights = new double[unvisited.size()];
        for (int i = 0; i < weights.length; i++) {
            weights[i] = unvisited.get(i).getWeight();
        }

        int depotReturns = 0;
        while (!unvisited.isEmpty()) {
            ServicePoint nextPoint = null;
            double minScore = Double.MAX_VALUE;
            double minEnergy = Double.MAX_VALUE;
            int nextIndex = -1;
            for (int i = 0; i < unvisited.size(); i++) {
                ServicePoint sp = unvisited.get(i);
                double dist = horizDist(current, sp);
                double energy = estimateEnergyCost(current, sp, totalWeight);
                double score = dist / weights[i];
                if (score < minScore) {
                    minScore = score;
                    minEnergy = energy;
                    nextPoint = sp;
                    nextIndex = i;
                }
            }
            if (battery - minEnergy < batteryThreshold) {
                // Return to depot safely.
                finalReturnToDepot(route, current, depot, maxAltitude);
                depotReturns++;
                battery = initialBattery * 3600.0;
                // Recharge and ascend.
                route.addAll(createBezierArc(
                        new Waypoint(depot.getX(), 0, depot.getZ(), "DepotBatteryReplaced"),
                        new Waypoint(depot.getX(), maxAltitude, depot.getZ(), "AscendAfterRecharge"),
                        ARC_SAMPLES));
                current = new Waypoint(depot.getX(), maxAltitude, depot.getZ(), "CruisingAltitude");
            }
            // Approach service point at cruising altitude.
            route.addAll(createBezierArc(current,
                    new Waypoint(nextPoint.getX(), maxAltitude, nextPoint.getZ(), "Approach"),
                    ARC_SAMPLES));
            current = new Waypoint(nextPoint.getX(), maxAltitude, nextPoint.getZ(), "AboveServicePoint");
            // Descend to 10 m for service.
            route.addAll(createBezierArc(current,
                    new Waypoint(nextPoint.getX(), 10.0, nextPoint.getZ(), "DescentToService"),
                    ARC_SAMPLES));
            // Add service delivery waypoint (carrying the delivered payload as weight).
            route.add(new Waypoint(nextPoint.getX(), 10.0, nextPoint.getZ(), "Hover30sService", weights[nextIndex]));
            battery -= minEnergy + HOVER_TIME * totalWeight * POWER_PER_KG;
            current = new Waypoint(nextPoint.getX(), 10.0, nextPoint.getZ(), "ServiceCompleted");
            // Ascend back to cruising altitude.
            route.addAll(createBezierArc(current,
                    new Waypoint(nextPoint.getX(), maxAltitude, nextPoint.getZ(), "AscendFromService"),
                    ARC_SAMPLES));
            current = new Waypoint(nextPoint.getX(), maxAltitude, nextPoint.getZ(), "CruisingAltitude");

            unvisited.remove(nextPoint);
            weights[nextIndex] = 0;
            for (int i = 0; i < unvisited.size(); i++) {
                weights[i] = Math.min(100, weights[i] + RAND.nextDouble() * 2);
            }
        }
        // Final safe return.
        finalReturnToDepot(route, current, depot, maxAltitude);
        route.add(new Waypoint(0, 0, 0, "DepotReturns", depotReturns));
        return route;
    }

    // ---------------- MST-TSP APPROXIMATION ----------------
    private static List<SpatialNode> mstTspApprox(Graph3D graph,
                                                  DroneNode depot,
                                                  List<ServicePoint> servicePoints,
                                                  double maxAltitude,
                                                  double initialBattery,
                                                  double totalWeight) {
        List<SpatialNode> route = new ArrayList<>();
        double battery = initialBattery * 3600.0;
        double batteryThreshold = initialBattery * BATTERY_THRESHOLD_PERCENT * 3600.0;

        SpatialNode current = new Waypoint(depot.getX(), depot.getY(), depot.getZ(), "DepotStart");
        route.add(current);

        route.addAll(createBezierArc(current,
                new Waypoint(depot.getX(), maxAltitude, depot.getZ(), "AscendToCruise"),
                ARC_SAMPLES));
        current = new Waypoint(depot.getX(), maxAltitude, depot.getZ(), "CruisingAltitude");

        List<SpatialNode> unvisited = new ArrayList<>(servicePoints);
        double[] weights = new double[unvisited.size()];
        for (int i = 0; i < weights.length; i++) {
            weights[i] = ((ServicePoint) unvisited.get(i)).getWeight();
        }

        int depotReturns = 0;
        while (!unvisited.isEmpty()) {
            List<SpatialNode> allNodes = new ArrayList<>();
            allNodes.add(depot);
            allNodes.addAll(unvisited);
            List<Edge> mstEdges = buildMST(allNodes, weights, unvisited);
            List<SpatialNode> nodeOrder = preOrderTraversal(depot, allNodes, mstEdges);
            for (int i = 1; i < nodeOrder.size(); i++) {
                SpatialNode nxt = nodeOrder.get(i);
                if (!(nxt instanceof ServicePoint)) continue;
                int idx = unvisited.indexOf(nxt);
                if (idx == -1) continue;
                double energyToNext = estimateEnergyCost(current, nxt, totalWeight);
                if (battery - energyToNext < batteryThreshold) {
                    finalReturnToDepot(route, current, depot, maxAltitude);
                    depotReturns++;
                    battery = initialBattery * 3600.0;
                    route.addAll(createBezierArc(
                            new Waypoint(depot.getX(), 0, depot.getZ(), "DepotBatteryReplaced"),
                            new Waypoint(depot.getX(), maxAltitude, depot.getZ(), "AscendAfterRecharge"),
                            ARC_SAMPLES));
                    current = new Waypoint(depot.getX(), maxAltitude, depot.getZ(), "CruisingAltitude");
                }
                route.addAll(createBezierArc(current,
                        new Waypoint(nxt.getX(), maxAltitude, nxt.getZ(), "Approach"),
                        ARC_SAMPLES));
                current = new Waypoint(nxt.getX(), maxAltitude, nxt.getZ(), "AboveServicePoint");
                route.addAll(createBezierArc(current,
                        new Waypoint(nxt.getX(), 10.0, nxt.getZ(), "DescentToService"),
                        ARC_SAMPLES));
                route.add(new Waypoint(nxt.getX(), 10.0, nxt.getZ(), "Hover30sService", weights[idx]));
                battery -= energyToNext + HOVER_TIME * totalWeight * POWER_PER_KG;
                current = new Waypoint(nxt.getX(), 10.0, nxt.getZ(), "ServiceCompleted");
                route.addAll(createBezierArc(current,
                        new Waypoint(nxt.getX(), maxAltitude, nxt.getZ(), "AscendFromService"),
                        ARC_SAMPLES));
                current = new Waypoint(nxt.getX(), maxAltitude, nxt.getZ(), "CruisingAltitude");

                unvisited.remove(nxt);
                weights[idx] = 0;
                for (int j = 0; j < unvisited.size(); j++) {
                    weights[j] = Math.min(100, weights[j] + RAND.nextDouble() * 2);
                }
            }
        }
        double energyToDepot = estimateEnergyCost(current, depot, totalWeight);
        if (battery - energyToDepot < batteryThreshold) {
            finalReturnToDepot(route, current, depot, maxAltitude);
            depotReturns++;
            battery = initialBattery * 3600.0;
            route.addAll(createBezierArc(
                    new Waypoint(depot.getX(), 0, depot.getZ(), "DepotBatteryReplaced"),
                    new Waypoint(depot.getX(), maxAltitude, depot.getZ(), "AscendAfterFinalReturn"),
                    ARC_SAMPLES));
            current = new Waypoint(depot.getX(), maxAltitude, depot.getZ(), "CruisingAltitude");
        }
        finalReturnToDepot(route, current, depot, maxAltitude);
        route.add(new Waypoint(0, 0, 0, "DepotReturns", depotReturns));
        return route;
    }

    /**
     * Enforces a strictly vertical and horizontal 4-step approach back to the depot.
     */
    private static void finalReturnToDepot(List<SpatialNode> route,
                                           SpatialNode current,
                                           DroneNode depot,
                                           double maxAltitude) {
        if (Math.abs(current.getY() - maxAltitude) > 1e-3) {
            route.addAll(createBezierArc(current,
                    new Waypoint(current.getX(), maxAltitude, current.getZ(), "VerticalToMaxAlt"),
                    ARC_SAMPLES));
        }
        SpatialNode step1 = new Waypoint(current.getX(), maxAltitude, current.getZ(), "VerticalToMaxAlt");
        if (Math.abs(step1.getX() - depot.getX()) > 1e-3 || Math.abs(step1.getZ() - depot.getZ()) > 1e-3) {
            route.addAll(createBezierArc(step1,
                    new Waypoint(depot.getX(), maxAltitude, depot.getZ(), "AboveDepot"),
                    ARC_SAMPLES));
        }
        SpatialNode step2 = new Waypoint(depot.getX(), maxAltitude, depot.getZ(), "AboveDepot");
        route.addAll(createBezierArc(step2,
                    new Waypoint(depot.getX(), 50, depot.getZ(), "DescendTo50m"),
                    ARC_SAMPLES));
        SpatialNode step3 = new Waypoint(depot.getX(), 50, depot.getZ(), "DescendTo50m");
        route.addAll(createBezierArc(step3,
                    new Waypoint(depot.getX(), 0, depot.getZ(), "DescendToZero"),
                    ARC_SAMPLES));
    }

    // ---------------- Quadratic Bézier Arc ----------------
    private static List<SpatialNode> createBezierArc(SpatialNode start, SpatialNode end, int numSamples) {
        List<SpatialNode> arcPoints = new ArrayList<>();
        double x0 = start.getX(), y0 = start.getY(), z0 = start.getZ();
        double x2 = end.getX(), y2 = end.getY(), z2 = end.getZ();

        boolean legitDescent = isLegitDescent(start) || isLegitDescent(end);
        if (!legitDescent) {
            if (y0 < 50) y0 = 50;
            if (y2 < 50) y2 = 50;
        }
        
        boolean forceHorizontal = false;
        if (start instanceof Waypoint && end instanceof Waypoint) {
            String sl = ((Waypoint) start).getLabel();
            String el = ((Waypoint) end).getLabel();
            if ((sl.contains("Approach") || sl.contains("CruisingAltitude") || sl.contains("AboveServicePoint") || sl.contains("AboveDepot")) &&
                (el.contains("Approach") || el.contains("CruisingAltitude") || el.contains("AboveServicePoint") || el.contains("AboveDepot"))) {
                forceHorizontal = true;
            }
        }
        
        if (forceHorizontal) {
            double fixedY = Math.max(y0, y2);
            double x1 = (x0 + x2) / 2.0;
            double z1 = (z0 + z2) / 2.0;
            for (int i = 1; i <= numSamples; i++) {
                double t = (double) i / numSamples;
                double oneMinusT = 1 - t;
                double x = oneMinusT * oneMinusT * x0 + 2 * oneMinusT * t * x1 + t * t * x2;
                double y = fixedY;
                double z = oneMinusT * oneMinusT * z0 + 2 * oneMinusT * t * z1 + t * t * z2;
                arcPoints.add(new Waypoint(x, y, z, "ArcSegment"));
            }
            return arcPoints;
        }
        
        double midX = (x0 + x2) / 2.0;
        double midZ = (z0 + z2) / 2.0;
        double midY = (y0 + y2) / 2.0;

        double controlY;
        if (Math.abs(y2 - y0) < 1e-3) {
            controlY = y0;
        } else if (y0 > y2) {
            controlY = midY - CURVATURE_FACTOR * (y0 - y2);
        } else {
            controlY = midY + CURVATURE_FACTOR * (y2 - y0);
        }
        double x1 = midX, y1 = controlY, z1 = midZ;

        for (int i = 1; i <= numSamples; i++) {
            double t = (double) i / numSamples;
            double oneMinusT = 1 - t;
            double x = oneMinusT * oneMinusT * x0 + 2 * oneMinusT * t * x1 + t * t * x2;
            double y = oneMinusT * oneMinusT * y0 + 2 * oneMinusT * t * y1 + t * t * y2;
            double z = oneMinusT * oneMinusT * z0 + 2 * oneMinusT * t * z1 + t * t * z2;
            if (!legitDescent && y < 50) y = 50;
            arcPoints.add(new Waypoint(x, y, z, "ArcSegment"));
        }
        return arcPoints;
    }

    /**
     * Checks if a node's label indicates a legitimate descent.
     */
    private static boolean isLegitDescent(SpatialNode node) {
        if (!(node instanceof Waypoint)) return false;
        String label = ((Waypoint) node).getLabel();
        return label.contains("DescentToService")
                || label.contains("Hover30sService")
                || label.contains("DepotBatteryReplaced")
                || label.contains("DescendTo50m")
                || label.contains("DescendToZero");
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
                            cost /= Math.max(1, weights[index]);
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
     * Calculates metrics for the route:
     * [0] Energy (Wh),
     * [1] Flight Time (s),
     * [2] Service Points Visited,
     * [3] Adaptability (%),
     * [4] Depot Returns,
     * [5] Total Distance (m)
     * 
     * This version simulates payload drop-off by updating currentPayload after each service delivery.
     */
    public static double[] calculateMetricsForRoute(List<SpatialNode> route, int totalPoints, double initialPayload) {
        double totalJoules = 0;
        double cumulativeFlightTime = 0;
        int pointsVisited = 0;
        double totalDistance = 0;
        List<Double> visitWeights = new ArrayList<>();
        int depotReturns = 0;
        
        // Dynamic payload: start with initial payload
        double currentPayload = initialPayload;

        for (int i = 0; i < route.size() - 1; i++) {
            SpatialNode from = route.get(i);
            SpatialNode to = route.get(i + 1);
            double dx = to.getX() - from.getX();
            double dy = to.getY() - from.getY();
            double dz = to.getZ() - from.getZ();
            double legDistance = Math.sqrt(dx * dx + dy * dy + dz * dz);
            totalDistance += legDistance;
            double horiz = Math.sqrt(dx * dx + dz * dz);
            double tHoriz = horiz / CRUISE_SPEED;
            double tVert = (Math.abs(dy) > 0.01) ? Math.abs(dy) / VERT_SPEED : 0;
            double flightTime = tHoriz + tVert;
            cumulativeFlightTime += flightTime;

            // Use dynamic payload for energy calculation:
            double totalW = DRONE_WEIGHT + currentPayload;
            double legJoules = flightTime * totalW * POWER_PER_KG;
            String label = (to instanceof Waypoint) ? ((Waypoint) to).getLabel() : "";
            if (label.contains("Hover30sService")) {
                legJoules += HOVER_TIME * totalW * POWER_PER_KG;
                pointsVisited++;
                if (to instanceof Waypoint) {
                    double delivered = ((Waypoint) to).getWeight();
                    visitWeights.add(delivered);
                    // Simulate drop-off: subtract delivered payload (not below zero)
                    currentPayload = Math.max(0, currentPayload - delivered);
                }
            } else if (label.contains("DepotBatteryReplaced")) {
                depotReturns++;
            }
            totalJoules += legJoules;
        }
        SpatialNode lastNode = route.get(route.size() - 1);
        if (lastNode instanceof Waypoint && ((Waypoint) lastNode).getLabel().equals("DepotReturns")) {
            depotReturns = (int)((Waypoint) lastNode).getWeight();
        }
        double adaptability = 0;
        if (pointsVisited > 0) {
            int firstHalf = (int)Math.ceil(pointsVisited / 2.0);
            int highPriorityCount = 0;
            for (int i = 0; i < Math.min(firstHalf, visitWeights.size()); i++) {
                if (visitWeights.get(i) >= 70) highPriorityCount++;
            }
            adaptability = (double) highPriorityCount / firstHalf * 100;
        }
        double totalWh = totalJoules / 3600.0;
        return new double[]{totalWh, cumulativeFlightTime, pointsVisited, adaptability, depotReturns, totalDistance};
    }

    public static double calculateMonetaryCost(double energyWh) {
        return (energyWh / 1000.0) * 0.13;
    }
}
