package drone;

/**
 * A refill station on ground (y=0).
 */
public class RendezvousPoint implements SpatialNode {
    private int id;
    private double x, y, z;

    public RendezvousPoint(int id, double x, double y, double z) {
        this.id = id;
        this.x = x;
        this.y = y; // ground
        this.z = z;
    }

    public int getId() { return id; }

    @Override
    public double getX() { return x; }
    @Override
    public double getY() { return y; }
    @Override
    public double getZ() { return z; }

    @Override
    public String toString() {
        return "Rendezvous(" + id + ") [" + x + ", " + y + ", " + z + "]";
    }
}




