package drone;

import java.util.*;

public class Graph3D {
    private double width, length, altitude;
    private double transmissionRange;
    private List<SpatialNode> nodes;

    public Graph3D(double width, double length, double altitude, double transmissionRange) {
        this.width = width;
        this.length = length;
        this.altitude = altitude;
        this.transmissionRange = transmissionRange;
        this.nodes = new ArrayList<>();
    }

    public void addNode(SpatialNode node) {
        nodes.add(node);
    }

    public double getWidth() { return width; }
    public double getLength() { return length; }
    public double getAltitude() { return altitude; }
    public List<SpatialNode> getNodes() { return nodes; }
}





