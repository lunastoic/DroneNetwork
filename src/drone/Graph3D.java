package drone;

import java.util.ArrayList;
import java.util.List;

/**
 * Represents a 3D graph structure for the simulation, containing nodes and dimensions.
 */
public class Graph3D {
    private double width;
    private double altitude;
    private double length;
    private List<SpatialNode> nodes;

    public Graph3D(double width, double altitude, double length) {
        this.width = width;
        this.altitude = altitude;
        this.length = length;
        this.nodes = new ArrayList<>();
    }

    public void addNode(SpatialNode node) {
        nodes.add(node);
    }

    public List<SpatialNode> getNodes() {
        return nodes;
    }

    public double getWidth() {
        return width;
    }

    public double getAltitude() {
        return altitude;
    }

    public double getLength() {
        return length;
    }
}
