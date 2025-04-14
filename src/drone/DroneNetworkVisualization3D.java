package drone;

import javafx.animation.AnimationTimer;
import javafx.application.Application;
import javafx.application.Platform;
import javafx.scene.*;
import javafx.scene.input.KeyEvent;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import javafx.scene.input.ScrollEvent;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.*;
import javafx.scene.text.Font;
import javafx.scene.text.Text;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Translate;
import javafx.stage.Stage;
import java.util.ArrayList;
import java.util.List;

/**
 * DroneNetworkVisualization3D:
 * - Draws a floor grid.
 * - Renders two flight paths: Greedy (default blue) and MST-TSP (default red), with MST route offset by +10m in X.
 * - Segments that are part of recharge legs (detected via node labels containing "Refill" or "DepotBatteryReplaced")
 *   are drawn in gray.
 */
public class DroneNetworkVisualization3D extends Application {

    private static Graph3D graph;
    private static List<SpatialNode> routeGreedy;
    private static List<SpatialNode> routeMST;
    private static boolean isRunning = false;

    // Camera parameters.
    private PerspectiveCamera camera;
    private double angleX = -45;
    private double angleY = 90;
    private double cameraDistance = 2000;
    private double pivotX = 0;
    private double pivotY = 0;
    private double pivotZ = 0;

    // Mouse interaction variables.
    private double anchorX, anchorY;
    private double anchorAngleX, anchorAngleY;
    private double anchorPanX, anchorPanZ;
    private double anchorDistance;
    private static final double ROTATION_SENSITIVITY = 0.1;
    private static final double PAN_SENSITIVITY = 0.2;
    private static final double ZOOM_SENSITIVITY = 0.2;

    // For labeling.
    private final List<Text> labelNodes = new ArrayList<>();

    /**
     * Launches the visualizer with the two routes.
     */
    public static void launchVisualizer(Graph3D g, List<SpatialNode> greedyRoute, List<SpatialNode> mstRoute) {
        if (g == null || greedyRoute == null || mstRoute == null) {
            System.out.println("ERROR: Visualization data is null! Cannot launch.");
            return;
        }
        graph = g;
        routeGreedy = greedyRoute;
        routeMST = mstRoute;
        System.out.println("Graph: width=" + graph.getWidth() +
                ", altitude=" + graph.getAltitude() +
                ", length=" + graph.getLength());
        System.out.println("Greedy route size: " + routeGreedy.size());
        System.out.println("MST route size: " + routeMST.size());
        if (!isRunning) {
            isRunning = true;
            Platform.startup(() -> {
                try {
                    new DroneNetworkVisualization3D().start(new Stage());
                } catch (Exception e) {
                    e.printStackTrace();
                }
            });
        } else {
            System.out.println("JavaFX is already running.");
        }
    }

    @Override
    public void start(Stage primaryStage) {
        Group root = new Group();

        // Basic lighting.
        AmbientLight ambient = new AmbientLight(Color.WHITE);
        PointLight light = new PointLight(Color.WHITE);
        light.setTranslateX(graph.getWidth() / 2.0);
        light.setTranslateY(graph.getAltitude() / 2.0);
        light.setTranslateZ(graph.getLength() / 2.0);
        root.getChildren().addAll(ambient, light);

        // Floor grid.
        Group floorGroup = createFloorGrid(graph.getWidth(), graph.getLength(), 50);
        root.getChildren().add(floorGroup);

        // Greedy route.
        Group greedyGroup = new Group();
        drawRoute(greedyGroup, routeGreedy, Color.BLUE);
        root.getChildren().add(greedyGroup);

        // MST route with an X offset of +10.
        Group mstGroup = new Group();
        drawRoute(mstGroup, routeMST, Color.RED);
        mstGroup.getTransforms().add(new Translate(10, 0, 0));
        root.getChildren().add(mstGroup);

        addSceneLabels(root);

        pivotX = 0;
        pivotY = 0;
        pivotZ = 0;
        camera = new PerspectiveCamera(true);
        camera.setNearClip(0.1);
        camera.setFarClip(1e6);
        updateCamera();

        Scene scene = new Scene(root, 1200, 800, true);
        scene.setFill(Color.LIGHTSKYBLUE);
        scene.setCamera(camera);

        scene.setOnMousePressed(this::handleMousePressed);
        scene.setOnMouseDragged(this::handleMouseDragged);
        scene.setOnScroll(this::handleScroll);
        scene.setOnKeyPressed(this::handleKeyPressed);

        primaryStage.setTitle("Drone Flight Path - Greedy (Blue) vs MST-TSP (Red)");
        primaryStage.setScene(scene);
        primaryStage.show();

        AnimationTimer timer = new AnimationTimer() {
            @Override
            public void handle(long now) {
                billboardLabels();
            }
        };
        timer.start();
    }

    private Group createFloorGrid(double width, double length, double spacing) {
        Group group = new Group();
        double thickness = 0.5;
        double floorY = 0;
        for (double z = 0; z <= length; z += spacing) {
            Box line = new Box(width, thickness, thickness);
            line.setTranslateX(width / 2.0);
            line.setTranslateY(floorY);
            line.setTranslateZ(z);
            line.setMaterial(new PhongMaterial(Color.BLACK));
            group.getChildren().add(line);
        }
        for (double x = 0; x <= width; x += spacing) {
            Box line = new Box(thickness, thickness, length);
            line.setTranslateX(x);
            line.setTranslateY(floorY);
            line.setTranslateZ(length / 2.0);
            line.setMaterial(new PhongMaterial(Color.BLACK));
            group.getChildren().add(line);
        }
        return group;
    }

    /**
     * Draws the route.
     * If a segment is part of a recharge (refill) leg (the destination node's label contains "Refill" or "DepotBatteryReplaced"),
     * that segment is drawn in gray.
     */
    private void drawRoute(Group parent, List<SpatialNode> route, Color defaultColor) {
        // Draw nodes.
        for (SpatialNode node : graph.getNodes()) {
            Sphere sphere = new Sphere(5);
            sphere.setTranslateX(node.getX());
            sphere.setTranslateY(node.getY());
            sphere.setTranslateZ(node.getZ());
            PhongMaterial mat = new PhongMaterial();
            if (node instanceof ServicePoint) {
                mat.setDiffuseColor(Color.RED);
            } else if (node instanceof DroneNode && node.getX() == 0 && node.getY() == 0 && node.getZ() == 0) {
                mat.setDiffuseColor(Color.BLUE);
            } else {
                mat.setDiffuseColor(defaultColor);
            }
            sphere.setMaterial(mat);
            parent.getChildren().add(sphere);
            addNodeLabel(parent, node);
        }
        // Draw route segments.
        if (route != null && route.size() > 1) {
            for (int i = 0; i < route.size() - 1; i++) {
                SpatialNode from = route.get(i);
                SpatialNode to = route.get(i + 1);
                Color segColor = (isRechargeSegment(from, to)) ? Color.GRAY : defaultColor;
                List<Cylinder> segs = createLineSegments3D(from, to, segColor, 2.0);
                parent.getChildren().addAll(segs);
            }
        }
    }

    private boolean isRechargeSegment(SpatialNode from, SpatialNode to) {
        String fromLabel = (from instanceof Waypoint) ? ((Waypoint) from).getLabel() : "";
        String toLabel = (to instanceof Waypoint) ? ((Waypoint) to).getLabel() : "";
        return (fromLabel.contains("Refill") || toLabel.contains("Refill") || toLabel.contains("DepotBatteryReplaced"));
    }

    private static final double SEGMENT_SPACING = 10.0;

    private List<Cylinder> createLineSegments3D(SpatialNode from, SpatialNode to, Color color, double radius) {
        List<Cylinder> cylinders = new ArrayList<>();
        double dx = to.getX() - from.getX();
        double dy = to.getY() - from.getY();
        double dz = to.getZ() - from.getZ();
        double dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
        if (dist < 1e-3) return cylinders;
        int numSegments = (int) Math.ceil(dist / SEGMENT_SPACING);
        for (int i = 0; i < numSegments; i++) {
            double t0 = (double) i / numSegments;
            double t1 = (double) (i + 1) / numSegments;
            double sx = from.getX() + t0 * dx;
            double sy = from.getY() + t0 * dy;
            double sz = from.getZ() + t0 * dz;
            double ex = from.getX() + t1 * dx;
            double ey = from.getY() + t1 * dy;
            double ez = from.getZ() + t1 * dz;
            Cylinder cyl = createOneCylinder(sx, sy, sz, ex, ey, ez, color, radius);
            if (cyl != null) {
                cylinders.add(cyl);
            }
        }
        return cylinders;
    }

    private Cylinder createOneCylinder(double x1, double y1, double z1,
                                       double x2, double y2, double z2,
                                       Color color, double radius) {
        double dx = x2 - x1;
        double dy = y2 - y1;
        double dz = z2 - z1;
        double length = Math.sqrt(dx * dx + dy * dy + dz * dz);
        if (length < 1e-3) return null;
        Cylinder cyl = new Cylinder(radius, length);
        cyl.setMaterial(new PhongMaterial(color));
        double midX = x1 + dx / 2.0;
        double midY = y1 + dy / 2.0;
        double midZ = z1 + dz / 2.0;
        cyl.setTranslateX(midX);
        cyl.setTranslateY(midY);
        cyl.setTranslateZ(midZ);
        if (Math.abs(dy) > 1e-3 || (Math.abs(dx) > 1e-3 && Math.abs(dz) > 1e-3)) {
            double thetaY = Math.toDegrees(Math.atan2(dx, dz));
            cyl.getTransforms().add(new Rotate(thetaY, Rotate.Y_AXIS));
            double horizDist = Math.sqrt(dx * dx + dz * dz);
            double thetaX = Math.toDegrees(Math.atan2(horizDist, dy)) - 90;
            cyl.getTransforms().add(new Rotate(thetaX, Rotate.X_AXIS));
        } else if (Math.abs(dz) > 1e-3) {
            cyl.getTransforms().add(new Rotate(90, Rotate.Y_AXIS));
        }
        return cyl;
    }

    private void addNodeLabel(Group parent, SpatialNode node) {
        int index = graph.getNodes().indexOf(node) + 1;
        String labelText = index + " (" +
                String.format("%.1f, %.1f, %.1f", node.getX(), node.getY(), node.getZ()) + ")";
        if (node instanceof ServicePoint) {
            double weight = ((ServicePoint) node).getWeight();
            labelText += " W:" + String.format("%.1f", weight);
        }
        Text txt = new Text(labelText);
        txt.setFont(Font.font(12));
        txt.setFill(Color.BLACK);
        txt.setTranslateX(node.getX() + 8);
        txt.setTranslateY(node.getY() - 8);
        txt.setTranslateZ(node.getZ());
        parent.getChildren().add(txt);
        labelNodes.add(txt);
    }

    private void addSceneLabels(Group parent) {
        Text origin = new Text("Origin(0,0,0)");
        origin.setFont(Font.font(12));
        origin.setFill(Color.BLACK);
        origin.setTranslateX(0);
        origin.setTranslateY(0);
        origin.setTranslateZ(0);
        parent.getChildren().add(origin);
        labelNodes.add(origin);

        Text corner = new Text(String.format("Max(%.0f, %.0f, %.0f)",
                graph.getWidth(), graph.getAltitude(), graph.getLength()));
        corner.setFont(Font.font(12));
        corner.setFill(Color.BLACK);
        corner.setTranslateX(graph.getWidth());
        corner.setTranslateY(graph.getAltitude());
        corner.setTranslateZ(graph.getLength());
        parent.getChildren().add(corner);
        labelNodes.add(corner);
    }

    private void updateCamera() {
        camera.getTransforms().clear();
        camera.getTransforms().add(new Rotate(180, Rotate.X_AXIS));
        camera.getTransforms().add(new Translate(-pivotX, -pivotY, -pivotZ));
        camera.getTransforms().add(new Rotate(angleY, Rotate.Y_AXIS));
        camera.getTransforms().add(new Rotate(angleX, Rotate.X_AXIS));
        camera.getTransforms().add(new Translate(0, 0, -cameraDistance));
    }

    private void billboardLabels() {
        for (Text txt : labelNodes) {
            txt.getTransforms().clear();
            txt.getTransforms().add(new Rotate(180, Rotate.X_AXIS));
            txt.getTransforms().add(new Rotate(-angleY, Rotate.Y_AXIS));
            txt.getTransforms().add(new Rotate(-angleX, Rotate.X_AXIS));
        }
    }

    private void handleMousePressed(MouseEvent e) {
        anchorX = e.getSceneX();
        anchorY = e.getSceneY();
        anchorAngleX = angleX;
        anchorAngleY = angleY;
        anchorPanX = pivotX;
        anchorPanZ = pivotZ;
        anchorDistance = cameraDistance;
    }

    private void handleMouseDragged(MouseEvent e) {
        double dx = e.getSceneX() - anchorX;
        double dy = e.getSceneY() - anchorY;
        if (e.getButton() == MouseButton.PRIMARY) {
            angleY = anchorAngleY + dx * ROTATION_SENSITIVITY;
            angleX = Math.max(-90, Math.min(90, anchorAngleX - dy * ROTATION_SENSITIVITY));
            updateCamera();
        } else if (e.getButton() == MouseButton.SECONDARY) {
            pivotX = anchorPanX - dx * PAN_SENSITIVITY;
            pivotZ = anchorPanZ - dy * PAN_SENSITIVITY;
            updateCamera();
        } else if (e.getButton() == MouseButton.MIDDLE) {
            cameraDistance = Math.max(50, Math.min(100000, anchorDistance - dy * ZOOM_SENSITIVITY * 10));
            updateCamera();
        }
    }

    private void handleScroll(ScrollEvent e) {
        double delta = e.getDeltaY() * ZOOM_SENSITIVITY;
        cameraDistance = Math.max(50, Math.min(100000, cameraDistance - delta));
        updateCamera();
    }

    private void handleKeyPressed(KeyEvent e) {
        switch (e.getCode()) {
            case LEFT:
                angleY -= 5 * ROTATION_SENSITIVITY;
                updateCamera();
                break;
            case RIGHT:
                angleY += 5 * ROTATION_SENSITIVITY;
                updateCamera();
                break;
            case UP:
                angleX = Math.max(-90, angleX - 5 * ROTATION_SENSITIVITY);
                updateCamera();
                break;
            case DOWN:
                angleX = Math.min(90, angleX + 5 * ROTATION_SENSITIVITY);
                updateCamera();
                break;
            case W:
                cameraDistance = Math.max(50, cameraDistance - 20 * ZOOM_SENSITIVITY);
                updateCamera();
                break;
            case S:
                cameraDistance = Math.min(100000, cameraDistance + 20 * ZOOM_SENSITIVITY);
                updateCamera();
                break;
        }
    }
}
