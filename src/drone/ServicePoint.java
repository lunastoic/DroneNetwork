package drone;

/**
 * Represents a service point in the 3D space, with an initial priority weight.
 */
public class ServicePoint extends SpatialNode {
    private final double weight; // Initial priority weight

    public ServicePoint(double x, double y, double z) {
        super(x, y, z);
        this.weight = 0; // Default weight
    }

    public ServicePoint(double x, double y, double z, double weight) {
        super(x, y, z);
        this.weight = weight;
    }

    /**
     * Gets the initial weight of the service point.
     * @return The weight (1-100).
     */
    public double getWeight() {
        return weight;
    }
}