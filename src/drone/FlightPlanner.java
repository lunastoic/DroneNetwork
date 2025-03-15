package drone;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Collectors;

public class FlightPlanner {
    private static final double DRONE_EMPTY = 10.0;   
    private static final double ONE_GALLON_KG = 3.785; 
    private static final double WATER_PER_FIRE = ONE_GALLON_KG;  
    private static final double HOVER_FIRE = 30.0;    
    private static final double HOVER_REFILL = 60.0;  
    private static final double ARC_PEAK = 50.0;      
    private static final int ARC_SAMPLES = 50;        
    private static final double CRUISE_SPEED = 15.0;  
    private static final double POWER_PER_KG = 200;   

    public static List<SpatialNode> planRoute(DroneNode depot,
                                              List<FireSite> fires,
                                              List<RendezvousPoint> rendezvous,
                                              double fullWaterKg) {
        List<SpatialNode> route = new ArrayList<>();
        double water = fullWaterKg;
        SpatialNode current = new Waypoint(depot.getX(), depot.getY(), depot.getZ(), "DepotStart");
        route.add(current);

        List<FireSite> sortedFires = fires.stream()
                .sorted(Comparator.comparingDouble(FireSite::getX))
                .collect(Collectors.toList());

        for (FireSite fire : sortedFires) {
            if (water < ONE_GALLON_KG) {
                RendezvousPoint refill = findNearestRefill(current, rendezvous);
                route.addAll(createArc(current, refill, ARC_PEAK, ARC_SAMPLES));
                route.add(new Waypoint(refill.getX(), refill.getY(), refill.getZ(), "Hover60sRefill"));
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

    private static List<SpatialNode> createArc(SpatialNode start, SpatialNode end,
                                              double arcPeak, int numSamples) {
        List<SpatialNode> arcPoints = new ArrayList<>();
        double x1 = start.getX(), y1 = start.getY(), z1 = start.getZ();
        double x2 = end.getX(), y2 = end.getY(), z2 = end.getZ();

        for (int i = 1; i <= numSamples; i++) {
            double t = (double)i / numSamples;
            double x = x1 + t * (x2 - x1);
            double z = z1 + t * (z2 - z1);
            double baseY = (1 - t) * y1 + t * y2;
            double offsetY = -4 * arcPeak * t * (1 - t);
            double y = baseY + offsetY;

            String segLabel = "ArcSegment";
            arcPoints.add(new Waypoint(x, y, z, segLabel));
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

    public static double computeEnergy(List<SpatialNode> route, double fullWaterKg) {
        double totalJ = 0;
        double water = fullWaterKg;
        double totalDistance = 0.0;
        int visitOrder = 0;
        double initialBattery = 3969.0; // Match DRONE_BATTERY from DroneFireSuppressionSim

        System.out.println("\n--- Mission Summary ---");
        for (int i = 0; i < route.size() - 1; i++) {
            SpatialNode from = route.get(i);
            SpatialNode to = route.get(i + 1);
            double dx = to.getX() - from.getX();
            double dy = to.getY() - from.getY();
            double dz = to.getZ() - from.getZ();
            double dist3D = Math.sqrt(dx * dx + dy * dy + dz * dz);
            totalDistance += dist3D; // Accumulate total distance

            double weight = DRONE_EMPTY + water;
            double legJ = 0.0;

            if (dist3D > 0.01) {
                double tFlight = dist3D / CRUISE_SPEED;
                legJ += tFlight * weight * POWER_PER_KG;
            }

            String toLabel = (to instanceof Waypoint) ? ((Waypoint) to).getLabel() : "";
            int timeSpent = 0;
            String nodeType = "";
            if (toLabel.contains("Hover30sFire")) {
                legJ += HOVER_FIRE * weight * POWER_PER_KG;
                water = Math.max(0, water - WATER_PER_FIRE);
                timeSpent = 30; // 30 seconds for fire nodes
                nodeType = "FireSite";
            } else if (toLabel.contains("Hover60sRefill")) {
                legJ += HOVER_REFILL * weight * POWER_PER_KG;
                water = fullWaterKg;
                timeSpent = 60; // 60 seconds (1 minute) for refill nodes
                nodeType = "RendezvousPoint";
            } else if (toLabel.contains("DepotStart") || toLabel.contains("ArcDepot")) {
                nodeType = "Depot";
            }

            if (nodeType != "") {
                visitOrder++;
                totalJ += legJ;
                double whConsumed = legJ / 3600.0;
                double batteryLeft = initialBattery - (totalJ / 3600.0);
                double payloadUsed = fullWaterKg - water;
                System.out.printf("Visit %d: %s at (%.1f, %.1f, %.1f), Battery Consumed: %.2f Wh, Battery Left: %.2f Wh, Payload Used: %.2f kg, Payload Left: %.2f kg, Time Spent: %d seconds\n",
                        visitOrder, nodeType, to.getX(), to.getY(), to.getZ(), whConsumed, batteryLeft, payloadUsed, water, timeSpent);
            }
        }

        // Final battery and total distance
        if (visitOrder > 0) {
            double batteryLeft = initialBattery - (totalJ / 3600.0);
            System.out.printf("Total Battery Consumed: %.2f Wh, Total Battery Left: %.2f Wh, Total Payload Left: %.2f kg, Total Distance Traveled: %.2f m\n",
                    totalJ / 3600.0, batteryLeft, water, totalDistance);
        }

        return totalJ;
    }
}
