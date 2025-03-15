 package drone;

/**
 * A sub-waypoint on the final 3D flight path (for arc sampling or hover).
 */
public class Waypoint implements SpatialNode {
    private double x, y, z;
    private String label;

    public Waypoint(double x, double y, double z, String label) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.label = label;
    }

    public String getLabel() { return label; }

    @Override
    public double getX() { return x; }
    @Override
    public double getY() { return y; }
    @Override
    public double getZ() { return z; }

    @Override
    public String toString() {
        return "Waypoint{" + label + " (" + x + ", " + y + ", " + z + ")}";
    }
}



