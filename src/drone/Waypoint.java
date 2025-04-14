package drone;

public class Waypoint extends SpatialNode {
    private String label;
    private double weight; // For service nodes: payload delivered (also used as priority)

    public Waypoint(double x, double y, double z, String label) {
        super(x, y, z);
        this.label = label;
        this.weight = 0; // default for non-service waypoints
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

    // This method returns the payload weight for service nodes;
    // it allows the simulation to subtract delivered payload.
    public double getPayloadWeight() {
        return weight;
    }
    
    public void setWeight(double weight) {
        this.weight = weight;
    }
}
