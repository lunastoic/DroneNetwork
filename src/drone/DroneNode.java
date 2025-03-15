package drone;

/**
 * The single drone, starts at (0,0,0) with a certain battery (Wh) and water payload (kg).
 */
public class DroneNode implements SpatialNode {
    private int id;
    private double x, y, z;
    private double batteryWh;
    private double waterKg;

    public DroneNode(int id, double x, double y, double z, double batteryWh, double waterKg) {
        this.id = id;
        this.x = x;
        this.y = y;
        this.z = z;
        this.batteryWh = batteryWh;
        this.waterKg = waterKg;
    }

    public int getId() { return id; }
    public double getBatteryWh() { return batteryWh; }
    public double getWaterKg() { return waterKg; }
    public void setWaterKg(double waterKg) { this.waterKg = waterKg; }

    @Override
    public double getX() { return x; }
    @Override
    public double getY() { return y; }
    @Override
    public double getZ() { return z; }

    @Override
    public String toString() {
        return "Drone(" + id + ") [" + x + ", " + y + ", " + z + 
               "], battery=" + batteryWh + " Wh, water=" + waterKg + " kg";
    }
}




