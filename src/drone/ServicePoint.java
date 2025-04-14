package drone;

/**
 * Represents a service point in 3D space with an associated weight, which acts as its priority.
 */
public class ServicePoint extends SpatialNode {
    private final double weight; // Higher weight means higher priority

    public ServicePoint(double x, double y, double z) {
        super(x, y, z);
        this.weight = 0;
    }

    public ServicePoint(double x, double y, double z, double weight) {
        super(x, y, z);
        this.weight = weight;
    }

    /**
     * Gets the initial priority weight.
     */
    public double getWeight() {
        return weight;
    }
}
