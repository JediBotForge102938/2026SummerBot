package org.firstinspires.ftc.teamcode.control;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import org.firstinspires.ftc.teamcode.BioBuzzLimelightClusterAdapter.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.ConfigurationSubsystem.StartingSpot;
import org.firstinspires.ftc.teamcode.subsystems.FieldMap;

/** Visibility-graph routes in field-center inches. */
public final class BioBuzzFieldMap {
    // Competition Manual 9.6.1 and 9.1: nominal frame and field tolerance.
    private static final double FRAME_HALF_WIDTH = 49.46 / 2;
    private static final double FRAME_HALF_DEPTH = 38.95 / 2;
    private static final double FIELD_TOLERANCE = 1;
    // Conservative field-plane bounds of the rings and their wall-side mounting hardware.
    private static final double FLOWER_HALF_ALONG_WALL = 3.35;
    private static final double FLOWER_DEPTH = 3.96 + 2.61;
    private static final double LANDMARK_STANDOFF = 3;
    private static final double GARDEN_WALL_CLEARANCE = 0.5;
    private static final double NODE_GAP = 0.1;
    private static final double EPSILON = 1e-8;

    private final double halfLength;
    private final double halfWidth;
    private final double transitLimit;
    private final List<Rect> geometry = new ArrayList<>();
    private final List<Rect> obstacles = new ArrayList<>();

    public static final class Destination {
        public final String name;
        public final FieldMap.Pose pose;

        public Destination(String name, FieldMap.Pose pose) {
            if (name == null || pose == null) throw new IllegalArgumentException("Destination is required");
            this.name = name;
            this.pose = pose;
        }
    }

    private static final class Rect {
        final double minX, maxX, minY, maxY;
        Rect(double minX, double maxX, double minY, double maxY) {
            this.minX = minX;
            this.maxX = maxX;
            this.minY = minY;
            this.maxY = maxY;
        }
    }

    public BioBuzzFieldMap(double lengthIn, double widthIn) {
        if (!Double.isFinite(lengthIn) || !Double.isFinite(widthIn)
                || lengthIn <= 0 || widthIn <= 0) {
            throw new IllegalArgumentException("Positive robot dimensions are required");
        }
        halfLength = lengthIn / 2;
        halfWidth = widthIn / 2;
        double radius = Math.hypot(halfLength, halfWidth);
        double inflation = radius + FIELD_TOLERANCE;
        transitLimit = FieldMap.HALF_FIELD - inflation;
        if (transitLimit <= 0) throw new IllegalArgumentException("Robot is too large for the field");
        FieldMap.Region frame = frameObstacle();
        geometry.add(new Rect(frame.minX, frame.maxX, frame.minY, frame.maxY));
        for (FieldMap.Region flower : flowerObstacles()) {
            geometry.add(new Rect(flower.minX, flower.maxX, flower.minY, flower.maxY));
        }

        for (Rect rect : geometry) {
            obstacles.add(expand(rect, inflation, inflation));
        }
    }

    public static List<FieldMap.Region> flowerObstacles() {
        List<FieldMap.Region> flowers = new ArrayList<>();
        for (FieldMap.Pose flower : FieldMap.FLOWER_WALL_POINTS) {
            boolean verticalWall = Math.abs(flower.x) == FieldMap.HALF_FIELD;
            double x = verticalWall ? flower.x - Math.signum(flower.x) * FLOWER_DEPTH / 2 : flower.x;
            double y = verticalWall ? flower.y : flower.y - Math.signum(flower.y) * FLOWER_DEPTH / 2;
            double hx = verticalWall ? FLOWER_DEPTH / 2 : FLOWER_HALF_ALONG_WALL;
            double hy = verticalWall ? FLOWER_HALF_ALONG_WALL : FLOWER_DEPTH / 2;
            flowers.add(new FieldMap.Region(x - hx, x + hx, y - hy, y + hy));
        }
        return Collections.unmodifiableList(flowers);
    }

    public static FieldMap.Region frameObstacle() {
        return new FieldMap.Region(-FRAME_HALF_WIDTH, FRAME_HALF_WIDTH,
                -FRAME_HALF_DEPTH, FRAME_HALF_DEPTH);
    }

    /** A standard wall start shifted toward field center if needed for flower clearance. */
    public FieldMap.Pose startingPose(Alliance alliance, StartingSpot side) {
        FieldMap.Pose pose = FieldMap.startingPose(alliance, side, halfLength * 2);
        double x = pose.x;
        for (FieldMap.Region flower : flowerObstacles()) {
            if (pose.y + halfLength >= flower.minY && pose.y - halfLength <= flower.maxY) {
                if (x > 0 && flower.minX > 0) {
                    x = Math.min(x, flower.minX - halfWidth - FIELD_TOLERANCE);
                } else if (x < 0 && flower.maxX < 0) {
                    x = Math.max(x, flower.maxX + halfWidth + FIELD_TOLERANCE);
                }
            }
        }
        return new FieldMap.Pose(x, pose.y, pose.headingDegrees);
    }

    /** Field landmarks with robot-center targets in the original menu order. */
    public List<Destination> destinations() {
        List<Destination> result = new ArrayList<>();
        String[] flowerNames = {"Red wall flower", "Blue wall flower",
                "Red audience flower", "Blue scoring flower"};
        for (FieldMap.Pose flower : FieldMap.FLOWER_WALL_POINTS) {
            boolean verticalWall = Math.abs(flower.x) == FieldMap.HALF_FIELD;
            double sign = Math.signum(verticalWall ? flower.x : flower.y);
            double inset = halfLength + FLOWER_DEPTH + FIELD_TOLERANCE;
            double x = verticalWall ? flower.x - sign * inset : flower.x;
            double y = verticalWall ? flower.y : flower.y - sign * inset;
            double heading = verticalWall ? (sign < 0 ? 180 : 0) : (sign < 0 ? -90 : 90);
            result.add(new Destination(flowerNames[result.size()], new FieldMap.Pose(x, y, heading)));
        }
        for (Alliance alliance : new Alliance[] {Alliance.RED, Alliance.BLUE}) {
            boolean red = alliance == Alliance.RED;
            FieldMap.Region garden = FieldMap.garden(alliance);
            double edgeY = red ? garden.maxY : garden.minY;
            double y = edgeY + (red ? 1 : -1) * (halfLength + LANDMARK_STANDOFF);
            double x = (red ? -1 : 1) * (FieldMap.HALF_FIELD - halfWidth - GARDEN_WALL_CLEARANCE);
            result.add(new Destination(red ? "Red garden" : "Blue garden",
                    new FieldMap.Pose(x, y, red ? -90 : 90)));
        }
        for (Alliance alliance : new Alliance[] {Alliance.RED, Alliance.BLUE}) {
            boolean red = alliance == Alliance.RED;
            FieldMap.Region parking = FieldMap.parking(alliance);
            double x = FieldMap.HALF_FIELD - halfLength - LANDMARK_STANDOFF;
            result.add(new Destination(red ? "Red parking" : "Blue parking",
                    new FieldMap.Pose(red ? -x : x, (parking.minY + parking.maxY) / 2,
                            red ? 180 : 0)));
        }
        for (Alliance alliance : new Alliance[] {Alliance.RED, Alliance.BLUE}) {
            for (StartingSpot side : new StartingSpot[] {StartingSpot.AUDIENCE, StartingSpot.SCORING}) {
                String color = alliance == Alliance.RED ? "Red" : "Blue";
                String startSide = side == StartingSpot.AUDIENCE ? "audience" : "scoring";
                result.add(new Destination(color + " " + startSide + " shooting start",
                        startingPose(alliance, side)));
            }
        }
        return Collections.unmodifiableList(result);
    }

    /** Returns waypoints after start, preserving heading near the walls. */
    public List<FieldMap.Pose> plan(FieldMap.Pose start, FieldMap.Pose goal) {
        if (start == null || goal == null) throw new IllegalArgumentException("Start and goal are required");
        if (!endpointClear(start) || !endpointClear(goal)) {
            throw new IllegalArgumentException("Start or goal overlaps a mapped obstacle or wall");
        }
        if (Math.hypot(start.x - goal.x, start.y - goal.y) < EPSILON) {
            if (Math.abs(Math.IEEEremainder(start.headingDegrees - goal.headingDegrees, 360)) > EPSILON) {
                throw new IllegalArgumentException("In-place heading changes are not supported");
            }
            return Collections.emptyList();
        }
        FieldMap.Pose entry = transitPoint(start);
        FieldMap.Pose exit = transitPoint(goal);
        if (!segmentClearOriented(start, entry) || !segmentClearOriented(goal, exit)) {
            throw new IllegalArgumentException("No clear straight departure or approach");
        }
        if (distance(entry, exit) < EPSILON
                && Math.abs(Math.IEEEremainder(start.headingDegrees - goal.headingDegrees, 360)) > EPSILON) {
            throw new IllegalArgumentException("No room to change heading on this route");
        }
        List<FieldMap.Pose> result = new ArrayList<>();
        if (distance(start, entry) > EPSILON) result.add(entry);
        List<FieldMap.Pose> transit = graphRoute(entry, exit, goal.headingDegrees);
        result.addAll(transit);
        if (distance(exit, goal) > EPSILON) result.add(goal);
        return Collections.unmodifiableList(result);
    }

    private boolean endpointClear(FieldMap.Pose p) {
        double h = Math.toRadians(p.headingDegrees);
        double xExtent = Math.abs(Math.cos(h)) * halfLength + Math.abs(Math.sin(h)) * halfWidth;
        double yExtent = Math.abs(Math.sin(h)) * halfLength + Math.abs(Math.cos(h)) * halfWidth;
        if (Math.abs(p.x) + xExtent > FieldMap.HALF_FIELD + EPSILON
                || Math.abs(p.y) + yExtent > FieldMap.HALF_FIELD + EPSILON) return false;
        for (Rect rect : geometry) {
            Rect expanded = expand(rect, xExtent, yExtent);
            if (inside(expanded, p.x, p.y)) return false;
        }
        return true;
    }

    private FieldMap.Pose transitPoint(FieldMap.Pose pose) {
        if (Math.abs(pose.x) < transitLimit && Math.abs(pose.y) < transitLimit
                && pointOutsideObstacles(pose.x, pose.y)) return pose;
        boolean vertical = Math.abs(pose.y) >= Math.abs(pose.x);
        double direction = vertical ? -Math.signum(pose.y) : -Math.signum(pose.x);
        for (double d = 0.25; d < FieldMap.HALF_FIELD * 2; d += 0.25) {
            double x = pose.x + (vertical ? 0 : d * direction);
            double y = pose.y + (vertical ? d * direction : 0);
            FieldMap.Pose candidate = new FieldMap.Pose(x, y, pose.headingDegrees);
            if (Math.abs(x) < transitLimit - NODE_GAP && Math.abs(y) < transitLimit - NODE_GAP
                    && pointOutsideObstacles(x, y) && segmentClearOriented(pose, candidate)) {
                return candidate;
            }
        }
        throw new IllegalArgumentException("No clear straight departure from the wall");
    }

    private boolean segmentClearOriented(FieldMap.Pose a, FieldMap.Pose b) {
        double h = Math.toRadians(a.headingDegrees);
        double x = Math.abs(Math.cos(h)) * halfLength + Math.abs(Math.sin(h)) * halfWidth;
        double y = Math.abs(Math.sin(h)) * halfLength + Math.abs(Math.cos(h)) * halfWidth;
        List<Rect> expanded = new ArrayList<>();
        for (Rect rect : geometry) expanded.add(expand(rect, x, y));
        return segmentClear(a, b, expanded);
    }

    private static Rect expand(Rect r, double x, double y) {
        return new Rect(r.minX - x, r.maxX + x, r.minY - y, r.maxY + y);
    }

    private static boolean inside(Rect r, double x, double y) {
        return x >= r.minX && x <= r.maxX && y >= r.minY && y <= r.maxY;
    }

    private List<FieldMap.Pose> graphRoute(FieldMap.Pose start, FieldMap.Pose goal, double heading) {
        List<FieldMap.Pose> nodes = new ArrayList<>();
        nodes.add(start);
        nodes.add(goal);
        for (Rect r : obstacles) {
            for (double x : new double[] {r.minX - NODE_GAP, r.maxX + NODE_GAP}) {
                for (double y : new double[] {r.minY - NODE_GAP, r.maxY + NODE_GAP}) {
                    if (Math.abs(x) <= transitLimit && Math.abs(y) <= transitLimit
                            && pointOutsideObstacles(x, y)) nodes.add(new FieldMap.Pose(x, y, heading));
                }
            }
        }
        double[] distances = new double[nodes.size()];
        Arrays.fill(distances, Double.POSITIVE_INFINITY);
        distances[0] = 0;
        int[] previous = new int[nodes.size()];
        Arrays.fill(previous, -1);
        boolean[] visited = new boolean[nodes.size()];
        for (int step = 0; step < nodes.size(); step++) {
            int current = -1;
            for (int i = 0; i < nodes.size(); i++) {
                if (!visited[i] && (current < 0 || distances[i] < distances[current])) current = i;
            }
            if (current < 0 || !Double.isFinite(distances[current])) break;
            visited[current] = true;
            for (int next = 0; next < nodes.size(); next++) {
                if (!visited[next] && segmentClear(nodes.get(current), nodes.get(next))) {
                    double candidate = distances[current] + distance(nodes.get(current), nodes.get(next));
                    if (candidate < distances[next]) {
                        distances[next] = candidate;
                        previous[next] = current;
                    }
                }
            }
        }
        if (!Double.isFinite(distances[1])) {
            throw new IllegalArgumentException("No route around mapped obstacles");
        }
        List<FieldMap.Pose> route = new ArrayList<>();
        for (int i = 1; i != 0; i = previous[i]) {
            FieldMap.Pose p = nodes.get(i);
            if (distance(p, start) > EPSILON) {
                route.add(new FieldMap.Pose(p.x, p.y, heading));
            }
        }
        Collections.reverse(route);
        return route;
    }

    private boolean pointOutsideObstacles(double x, double y) {
        for (Rect r : obstacles) {
            if (inside(r, x, y)) return false;
        }
        return true;
    }

    private boolean segmentClear(FieldMap.Pose a, FieldMap.Pose b) {
        return segmentClear(a, b, obstacles);
    }

    private static boolean segmentClear(FieldMap.Pose a, FieldMap.Pose b, List<Rect> rectangles) {
        for (Rect r : rectangles) {
            double enter = 0, exit = 1;
            double[] origins = {a.x, a.y};
            double[] delta = {b.x - a.x, b.y - a.y};
            double[] minimum = {r.minX, r.minY};
            double[] maximum = {r.maxX, r.maxY};
            for (int axis = 0; axis < 2; axis++) {
                if (Math.abs(delta[axis]) < EPSILON) {
                    if (origins[axis] < minimum[axis] || origins[axis] > maximum[axis]) {
                        enter = 2;
                        break;
                    }
                } else {
                    double t1 = (minimum[axis] - origins[axis]) / delta[axis];
                    double t2 = (maximum[axis] - origins[axis]) / delta[axis];
                    enter = Math.max(enter, Math.min(t1, t2));
                    exit = Math.min(exit, Math.max(t1, t2));
                }
            }
            if (enter <= exit) return false;
        }
        return true;
    }

    private static double distance(FieldMap.Pose a, FieldMap.Pose b) {
        return Math.hypot(a.x - b.x, a.y - b.y);
    }
}
