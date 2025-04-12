package drone;

/**
 * Abstract base class for nodes in 3D space, providing X, Y, Z coordinates.
 */
public abstract class SpatialNode {
    private double x, y, z;

    public SpatialNode(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getZ() {
        return z;
    }
}