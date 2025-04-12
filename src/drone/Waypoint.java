package drone;

public class Waypoint extends SpatialNode {
    private String label;
    private double weight; // Represents the service node's payload delivery amount (if applicable)

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

    // Now returns the stored weight (which is used for simulating payload drop-off)
    public double getPayloadWeight() {
        return weight;
    }
}
