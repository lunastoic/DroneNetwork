package drone;

public class Waypoint extends SpatialNode {
    private String label;
    private double weight; // Weight at the time of visit (for adaptability)

    public Waypoint(double x, double y, double z, String label) {
        super(x, y, z);
        this.label = label;
        this.weight = 0; // Default weight for non-service waypoints
    }

    public Waypoint(double x, double y, double z, String label, double weight) {
        super(x, y, z);
        this.label = label;
        this.weight = weight;
    }

    public String getLabel() {
        return label;
    }

    public double getWeight() {
        return weight;
    }

    // Add a method to get payload weight (for energy calculations)
    public double getPayloadWeight() {
        // This is a simplification; in a real scenario, payload might vary per waypoint
        // For now, assume payload is set by the caller (DroneEmergencySim)
        return 0; // Will be set by DroneEmergencySim via totalWeight
    }
}