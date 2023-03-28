package frc.robot.lib.PathFinder;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.opencv.calib3d.StereoBM;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory.State;
import frc.robot.lib.PathFinder.AABB;
import frc.robot.lib.PathFinder.FIeldConstatnts.FieldConstants;

public class PathFinder {

    public static void main(String[] args) throws IOException {
        // FIELD SETUP
        Node p1 = new Node(2, 0.5);
        Node p2 = new Node(2, 2.5);
        Node p3 = new Node(2, 5);
        Node p4 = new Node(5, 5);
        Node p5 = new Node(7, 2);
        Node p6 = new Node(4, 0.5);
        Node p7 = new Node(8, 3);

        p1.Neighbors = new ArrayList<Node>(Arrays.asList(p2, p6));
        p2.Neighbors = new ArrayList<Node>(Arrays.asList(p1, p3));
        p3.Neighbors = new ArrayList<Node>(Arrays.asList(p2, p4));
        p4.Neighbors = new ArrayList<Node>(Arrays.asList(p3, p7, p5));
        p5.Neighbors = new ArrayList<Node>(Arrays.asList(p4, p7, p6));
        p6.Neighbors = new ArrayList<Node>(Arrays.asList(p2, p5));
        p7.Neighbors = new ArrayList<Node>(Arrays.asList(p4, p5));

        AABB collisionTest = new AABB(FieldConstants.Community.chargingStationCorners[1],
                FieldConstants.Community.chargingStationCorners[3]);

        Graph graph = new Graph(new ArrayList<Node>(Arrays.asList(p1, p2, p3, p4, p5, p6, p7)));

        // PATH GENERATION
        Node robot = new Node(4, 4.25);
        Node target = p6;
        robot.Neighbors = new ArrayList<Node>(
                Arrays.asList(graph.findStart(robot, Arrays.asList(collisionTest), target)));
        var path = graph.aStar(robot, target);
        path = smooth(equalSpacingPath(path));

        var x = PathPlanner.generatePath(new PathConstraints(4, 3),
                convertNodeToPathPoints(path));

    }

    private static void saveJson(PathPlannerTrajectory x) throws IOException {
        JSONArray jsonArray = new JSONArray();
        JSONObject js;
        for (State state : x.getStates()) {
            js = new JSONObject();
            js.put("time", state.timeSeconds);
            js.put("pose", poseToJson(state.poseMeters));
            js.put("velocity", state.velocityMetersPerSecond);
            js.put("acceleration", state.accelerationMetersPerSecondSq);
            js.put("curvature", state.curvatureRadPerMeter);
            js.put("holonomicRotation", 0.0);
            js.put("angularVelocity", 0.0);
            js.put("holonomicAngularVelocity", 0.0);
            jsonArray.add(js);
        }
        var file = new FileWriter("src\\main\\deploy\\pathplanner\\generatedJSON\\test.wpilib.json");
        file.write(jsonArray.toJSONString());
        file.close();
    }

    private static JSONObject poseToJson(Pose2d state) {
        JSONObject returnObject = new JSONObject();
        JSONObject rotation = new JSONObject();
        JSONObject translation = new JSONObject();
        rotation.put("radians", state.getRotation().getRadians());
        translation.put("x", state.getTranslation().getX());
        translation.put("y", state.getTranslation().getY());

        returnObject.put("rotation", rotation);
        returnObject.put("translation", translation);

        return returnObject;
    }

    public static List<Node> equalSpacingPath(List<Node> points) {
        double spacing = 0.1;
        double current_distance = 0.0;
        int current_idx = 0;

        List<Node> new_points = new ArrayList<>();

        while (current_distance < points.get(points.size() - 1).g) {
            if (current_idx < points.size()) {
                if (current_distance > points.get(current_idx + 1).g) {
                    current_idx += 1;
                }
            }
            double progress = 1 - (points.get(current_idx + 1).g - current_distance)
                    / (points.get(current_idx + 1).g - points.get(current_idx).g);
            double x = points.get(current_idx).x
                    + (points.get(current_idx + 1).x - points.get(current_idx).x) * progress;
            double y = points.get(current_idx).y
                    + (points.get(current_idx + 1).y - points.get(current_idx).y) * progress;
            new_points.add(new Node(x, y));
            // System.out.println(new_points.get(new_points.size()-1).x+","+new_points.get(new_points.size()-1).y);
            current_distance += spacing;
        }

        return new_points;
    }

    public static List<Node> removeDup(List<Node> points) {
        Node lp = points.get(0);
        List<Node> returnPath = new ArrayList<>();
        returnPath.add(points.get(0));
        for (int i = 1; i < points.size(); i++) {
            if (!points.get(i).equals(lp)) {
                returnPath.add(points.get(i));
            }
            lp = points.get(i);
        }
        return returnPath;
    }

    public static ArrayList<Node> smooth(List<Node> path) {
        ArrayList<Node> returnPath = new ArrayList<>();
        int strentgh = 10;
        for (int i = 0; i < strentgh - 1; i++) {
            path.add(0, path.get(0));
        }
        for (int i = 0; i < path.size(); i++) {
            double sumx = 0, sumy = 0, count = 0;
            for (Node next : path.subList(i, Math.min(i + strentgh, path.size()))) {
                sumx += next.x;
                sumy += next.y;
                count++;
            }
            returnPath.add(new Node(sumx / count, sumy / count));
        }
        return returnPath;
    }

    public static void printDesmos(PathPlannerTrajectory x) {
        String your_mama = "";
        for (State y : x.getStates()) {
            your_mama += y.poseMeters.getX() + "," + y.poseMeters.getY() + "\n";
        }
        try (FileWriter fw = new FileWriter("src\\main\\java\\frc\\robot\\lib\\PathFinder\\path.txt")) {
            fw.write(your_mama);
            fw.close();
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }

    public static void printJulia(PathPlannerTrajectory x) {
        String your_mama = "x_array = [";
        for (State y : x.getStates()) {
            your_mama += y.poseMeters.getX() + ",";
        }
        your_mama += "]\ny_array = [";
        for (State y : x.getStates()) {
            your_mama += y.poseMeters.getY() + ",";
        }
        your_mama += "]";

        try (FileWriter fw = new FileWriter("src\\main\\java\\frc\\robot\\lib\\PathFinder\\path.txt")) {
            fw.write(your_mama);
            fw.close();
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }

    public static void printDesmos(List<Node> x) {
        String your_mama = "";
        for (Node y : x) {
            your_mama += y.x + "," + y.y + "\n";
        }
        try (FileWriter fw = new FileWriter("src\\main\\java\\frc\\robot\\lib\\PathFinder\\desmospath.txt")) {
            fw.write(your_mama);
            fw.close();
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }

    public static ArrayList<PathPoint> convertNodeToPathPoints(ArrayList<Node> path) {
        ArrayList<PathPoint> returnPath = new ArrayList<PathPoint>();
        double heading = 0;
        double last_heading = Math.atan2(path.get(0 + 1).y - path.get(0).y, path.get(0 + 1).x - path.get(0).x);
        for (int i = 0; i < path.size(); i++) {
            if (i < path.size() - 1) {
                heading = (Math.atan2(path.get(i + 1).y - path.get(i).y, path.get(i + 1).x - path.get(i).x)
                        + last_heading) / 2;
                last_heading = heading;
            }
            returnPath.add(
                    new PathPoint(new Translation2d(path.get(i).x, path.get(i).y), Rotation2d.fromRadians(heading)));
        }
        return returnPath;
    }
}
