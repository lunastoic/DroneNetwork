package drone;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.geometry.Insets;
import javafx.scene.*;
import javafx.scene.input.*;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Box;
import javafx.scene.shape.Cylinder;
import javafx.scene.shape.Sphere;
import javafx.scene.text.Font;
import javafx.scene.text.Text;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Translate;
import javafx.stage.Stage;

import java.util.List;

public class DroneNetworkVisualization3D extends Application {

    private static Graph3D graph;
    private static List<SpatialNode> route;
    private static boolean isRunning = false;

    // Camera orbit parameters
    private PerspectiveCamera camera;
    private double cameraDistance = 600; // Initial distance
    private double angleX = -45; // Initial vertical angle
    private double angleY = 0;   // Initial horizontal angle
    private double pivotX, pivotY, pivotZ; // Pivot point

    // For dragging
    private double anchorX, anchorY;
    private double anchorAngleX, anchorAngleY;
    private double anchorPanX, anchorPanZ;
    private double anchorDistance; // For zoom with middle click

    // Constants from FlightPlanner for energy simulation (if needed)
    private static final double DRONE_EMPTY = 10.0;
    private static final double ONE_GALLON_KG = 3.785;
    private static final double WATER_PER_FIRE = ONE_GALLON_KG;
    private static final double HOVER_FIRE = 30.0;
    private static final double HOVER_REFILL = 60.0;
    private static final double CRUISE_SPEED = 15.0;
    private static final double POWER_PER_KG = 200;
    private static final double FULL_WATER_KG = 8 * ONE_GALLON_KG;
    private static final double ROTATION_SENSITIVITY = 0.1; // Reduced sensitivity for smoother rotation
    private static final double PAN_SENSITIVITY = 0.2;      // Reduced sensitivity for smoother panning
    private static final double ZOOM_SENSITIVITY = 0.2;     // Granular zoom control

    public static void launchVisualizer(Graph3D g, List<SpatialNode> r) {
        if (g == null || r == null) {
            System.out.println("ERROR: Visualization data is null! Cannot launch.");
            return;
        }
        graph = g;
        route = r;
        System.out.println("Graph dimensions: width=" + graph.getWidth() + ", altitude=" + graph.getAltitude() + ", length=" + graph.getLength());
        System.out.println("Route size: " + route.size());
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

        // Add lighting
        AmbientLight ambient = new AmbientLight(Color.WHITE);
        PointLight light = new PointLight(Color.WHITE);
        light.setTranslateX(graph.getWidth() / 2);
        light.setTranslateY(graph.getAltitude() / 2);
        light.setTranslateZ(graph.getLength() / 2);
        root.getChildren().addAll(ambient, light);

        // 1) Bottom floor at y = 0 with 50m x 50m squares
        Group bottomFloor = createBottomFloor(graph.getWidth(), graph.getLength(), 50);
        root.getChildren().add(bottomFloor);

        // 2) Bounding box (only horizontal edges)
        double width = graph.getWidth();
        double height = graph.getAltitude();
        double length = graph.getLength();
        Group boundingBoxGroup = new Group();
        // Bottom face
        boundingBoxGroup.getChildren().add(createLine3D(0, 0, 0, width, 0, 0, Color.GRAY, 0.5));
        boundingBoxGroup.getChildren().add(createLine3D(width, 0, 0, width, 0, length, Color.GRAY, 0.5));
        boundingBoxGroup.getChildren().add(createLine3D(width, 0, length, 0, 0, length, Color.GRAY, 0.5));
        boundingBoxGroup.getChildren().add(createLine3D(0, 0, length, 0, 0, 0, Color.GRAY, 0.5));
        // Top face
        boundingBoxGroup.getChildren().add(createLine3D(0, height, 0, width, height, 0, Color.GRAY, 0.5));
        boundingBoxGroup.getChildren().add(createLine3D(width, height, 0, width, height, length, Color.GRAY, 0.5));
        boundingBoxGroup.getChildren().add(createLine3D(width, height, length, 0, height, length, Color.GRAY, 0.5));
        boundingBoxGroup.getChildren().add(createLine3D(0, height, length, 0, height, 0, Color.GRAY, 0.5));
        root.getChildren().add(boundingBoxGroup);

        addAxisLabels(root);

        // 3) Draw nodes (forcing them to the ground plane, y=0)
        for (SpatialNode node : graph.getNodes()) {
            Sphere sphere = new Sphere(5);
            sphere.setTranslateX(node.getX());
            sphere.setTranslateY(0); // Force y = 0
            sphere.setTranslateZ(node.getZ());

            PhongMaterial mat = new PhongMaterial();
            if (node instanceof FireSite) {
                mat.setDiffuseColor(Color.RED);
            } else if (node instanceof DroneNode && node.getX() == 0 && node.getY() == 0 && node.getZ() == 0) {
                mat.setDiffuseColor(Color.BLUE);
            } else if (node instanceof RendezvousPoint) {
                mat.setDiffuseColor(Color.ORANGE);
            } else {
                mat.setDiffuseColor(Color.BLUE);
            }
            String label = String.valueOf(graph.getNodes().indexOf(node) + 1);
            sphere.setMaterial(mat);

            final SpatialNode finalNode = node;
            sphere.setOnMouseClicked(event -> {
                if (event.getButton() == MouseButton.PRIMARY) {
                    lookAtNode(finalNode);
                }
            });

            root.getChildren().add(sphere);
            addLabel(root, node, label);
        }

        // 4) Draw edges based on the route (sequential connections)
        int edgeCount = 0;
        if (route != null && !route.isEmpty()) {
            for (int i = 0; i < route.size() - 1; i++) {
                SpatialNode from = route.get(i);
                SpatialNode to = route.get(i + 1);
                Cylinder edge = createLine3D(from.getX(), 0, from.getZ(), to.getX(), 0, to.getZ(), Color.GRAY, 0.2);
                if (edge != null) {
                    root.getChildren().add(edge);
                    edgeCount++;
                }
            }
        }
        System.out.println("Total edges added: " + edgeCount);

        // 5) Draw route as a continuous flight path
        int lineCount = 0;
        if (route != null && !route.isEmpty()) {
            for (int i = 0; i < route.size() - 1; i++) {
                SpatialNode from = route.get(i);
                SpatialNode to = route.get(i + 1);
                double fromY = (from instanceof Waypoint) ? from.getY() : 0;
                double toY = (to instanceof Waypoint) ? to.getY() : 0;
                Cylinder cyl = createLine3D(from.getX(), fromY, from.getZ(), to.getX(), toY, to.getZ(), Color.DODGERBLUE, 3.0);
                if (cyl != null) {
                    root.getChildren().add(cyl);
                    lineCount++;
                }
            }
        }
        System.out.println("Total route lines added: " + lineCount);

        // 6) Setup camera pivot at ground center
        pivotX = graph.getWidth() / 2;
        pivotY = 0;
        pivotZ = graph.getLength() / 2;

        camera = new PerspectiveCamera(true);
        camera.setNearClip(0.1);
        camera.setFarClip(100000);
        updateCamera();

        Scene scene = new Scene(root, 1200, 800, true);
        scene.setFill(Color.LIGHTSKYBLUE);
        scene.setCamera(camera);

        scene.setOnMousePressed(this::handleMousePressed);
        scene.setOnMouseDragged(this::handleMouseDragged);
        scene.setOnScroll(this::handleScroll);
        scene.setOnKeyPressed(this::handleKeyPressed);

        primaryStage.setTitle("3D Drone Flight Path - Top-Down + Node Click Teleport");
        primaryStage.setScene(scene);
        primaryStage.show();
    }

    // ========== CAMERA CONTROL METHODS ==========

    private void updateCamera() {
        camera.getTransforms().clear();
        camera.getTransforms().add(new Translate(-pivotX, -pivotY, -pivotZ));
        camera.getTransforms().add(new Rotate(angleY, Rotate.Y_AXIS));
        camera.getTransforms().add(new Rotate(angleX, Rotate.X_AXIS));
        camera.getTransforms().add(new Translate(0, 0, -cameraDistance));
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
            cameraDistance = Math.max(50, Math.min(10000, anchorDistance - dy * ZOOM_SENSITIVITY * 10));
            updateCamera();
        }
    }

    private void handleScroll(ScrollEvent e) {
        double delta = e.getDeltaY() * ZOOM_SENSITIVITY;
        cameraDistance = Math.max(50, Math.min(10000, cameraDistance - delta));
        updateCamera();
    }

    private void handleKeyPressed(KeyEvent e) {
        switch (e.getCode()) {
            case LEFT:
                angleY -= ROTATION_SENSITIVITY * 5;
                updateCamera();
                break;
            case RIGHT:
                angleY += ROTATION_SENSITIVITY * 5;
                updateCamera();
                break;
            case UP:
                angleX = Math.max(-90, angleX - ROTATION_SENSITIVITY * 5);
                updateCamera();
                break;
            case DOWN:
                angleX = Math.min(90, angleX + ROTATION_SENSITIVITY * 5);
                updateCamera();
                break;
            case W:
                cameraDistance = Math.max(50, cameraDistance - ZOOM_SENSITIVITY * 20);
                updateCamera();
                break;
            case S:
                cameraDistance = Math.min(10000, cameraDistance + ZOOM_SENSITIVITY * 20);
                updateCamera();
                break;
            default:
                break;
        }
    }

    /**
     * Teleports the camera pivot to the selected node.
     */
    private void lookAtNode(SpatialNode node) {
        pivotX = node.getX();
        pivotY = 0;
        pivotZ = node.getZ();
        updateCamera();
    }

    // ========== GRID + UTILITIES ==========

    private Group createBottomFloor(double w, double l, double spacing) {
        Group group = new Group();
        double thickness = 1.0;
        double y = 0;
        for (double z = 0; z <= l; z += spacing) {
            Box line = new Box(w, thickness, thickness);
            line.setTranslateX(w / 2);
            line.setTranslateY(y);
            line.setTranslateZ(z);
            line.setMaterial(new PhongMaterial(Color.BLACK));
            group.getChildren().add(line);
        }
        for (double x = 0; x <= w; x += spacing) {
            Box line = new Box(thickness, thickness, l);
            line.setTranslateX(x);
            line.setTranslateY(y);
            line.setTranslateZ(l / 2);
            line.setMaterial(new PhongMaterial(Color.BLACK));
            group.getChildren().add(line);
        }
        return group;
    }

    private Cylinder createLine3D(double x1, double y1, double z1,
                                  double x2, double y2, double z2,
                                  Color color, double radius) {
        double dx = x2 - x1;
        double dy = y2 - y1;
        double dz = z2 - z1;
        double length = Math.sqrt(dx * dx + dy * dy + dz * dz);
        if (length <= 0) return null;
        Cylinder cyl = new Cylinder(radius, length);
        cyl.setMaterial(new PhongMaterial(color));
        double midX = x1 + dx / 2;
        double midY = y1 + dy / 2;
        double midZ = z1 + dz / 2;
        cyl.setTranslateX(midX);
        cyl.setTranslateY(midY);
        cyl.setTranslateZ(midZ);
        if (dy != 0 || (dx != 0 && dz != 0)) {
            double thetaY = Math.toDegrees(Math.atan2(dx, dz));
            cyl.getTransforms().add(new Rotate(thetaY, Rotate.Y_AXIS));
            double horizDist = Math.sqrt(dx * dx + dz * dz);
            double thetaX = Math.toDegrees(Math.atan2(horizDist, dy)) - 90;
            cyl.getTransforms().add(new Rotate(thetaX, Rotate.X_AXIS));
        } else if (dz != 0) {
            cyl.getTransforms().add(new Rotate(90, Rotate.Y_AXIS));
        }
        return cyl;
    }

    private void addAxisLabels(Group root) {
        Text origin = new Text("Origin(0,0,0)");
        origin.setFont(Font.font(12));
        origin.setFill(Color.BLACK);
        origin.setTranslateX(0);
        origin.setTranslateY(0);
        origin.setTranslateZ(0);
        root.getChildren().add(origin);
        Text corner = new Text(String.format("Max(%.0f, %.0f, %.0f)",
                graph.getWidth(), graph.getAltitude(), graph.getLength()));
        corner.setFont(Font.font(12));
        corner.setFill(Color.BLACK);
        corner.setTranslateX(graph.getWidth());
        corner.setTranslateY(graph.getAltitude());
        corner.setTranslateZ(graph.getLength());
        root.getChildren().add(corner);
    }

    private void addLabel(Group root, SpatialNode node, String lbl) {
        Text txt = new Text(lbl + String.format(" (%.1f, %.1f, %.1f)",
                node.getX(), node.getY(), node.getZ()));
        txt.setFont(Font.font(12));
        txt.setFill(Color.BLACK);
        txt.setTranslateX(node.getX() + 8);
        txt.setTranslateY(0 - 8);
        txt.setTranslateZ(node.getZ());
        root.getChildren().add(txt);
    }
}
