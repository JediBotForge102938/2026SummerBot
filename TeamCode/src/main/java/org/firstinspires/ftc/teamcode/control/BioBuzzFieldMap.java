package org.firstinspires.ftc.teamcode.control;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/** Inches from field center: +X toward the blue wall, +Y away from the audience. */
public final class BioBuzzFieldMap {
    public static final double FIELD_HALF_SIZE = 72;
    public static final double ROBOT_HALF_SIZE = 9;
    public static final double ROBOT_RADIUS = Math.hypot(ROBOT_HALF_SIZE, ROBOT_HALF_SIZE);
    public static final double CLEARANCE = 2;
    // FIRST BIOBUZZ manual TU08, section 9.6.1 / figure 9-8, pp. 69-70.
    public static final double FRAME_HALF_WIDTH = 49.46 / 2;
    public static final double FRAME_HALF_DEPTH = 38.95 / 2;
    public static final double KEEP_OUT_X = FRAME_HALF_WIDTH + ROBOT_RADIUS + CLEARANCE;
    public static final double KEEP_OUT_Y = FRAME_HALF_DEPTH + ROBOT_RADIUS + CLEARANCE;
    // Reserve the entire perimeter strip for flowers, rather than estimating their centers.
    public static final double PERIMETER_STRIP = 6;
    public static final double CENTER_LIMIT =
            FIELD_HALF_SIZE - PERIMETER_STRIP - ROBOT_RADIUS - CLEARANCE;

    public static final class Point {
        public final double x;
        public final double y;

        public Point(double x, double y) {
            this.x = x;
            this.y = y;
        }

        public double distanceTo(Point other) {
            return Math.hypot(x - other.x, y - other.y);
        }
    }

    public boolean isClear(Point p) {
        return Double.isFinite(p.x) && Double.isFinite(p.y)
                && Math.abs(p.x) <= CENTER_LIMIT && Math.abs(p.y) <= CENTER_LIMIT
                && (Math.abs(p.x) > KEEP_OUT_X || Math.abs(p.y) > KEEP_OUT_Y);
    }

    public boolean segmentClear(Point a, Point b) {
        if (!isClear(a) || !isClear(b)) {
            return false;
        }
        double enter = 0;
        double exit = 1;
        double[] origin = {a.x, a.y};
        double[] delta = {b.x - a.x, b.y - a.y};
        double[] halfSize = {KEEP_OUT_X, KEEP_OUT_Y};
        for (int axis = 0; axis < 2; axis++) {
            if (Math.abs(delta[axis]) < 1e-9) {
                if (Math.abs(origin[axis]) > halfSize[axis]) {
                    return true;
                }
            } else {
                double t1 = (-halfSize[axis] - origin[axis]) / delta[axis];
                double t2 = (halfSize[axis] - origin[axis]) / delta[axis];
                enter = Math.max(enter, Math.min(t1, t2));
                exit = Math.min(exit, Math.max(t1, t2));
                if (enter > exit) {
                    return true;
                }
            }
        }
        return false;
    }

    /** A visibility graph around the entire frame; never attempts to pass under the hives. */
    public List<Point> plan(Point start, Point goal) {
        if (!isClear(start) || !isClear(goal)) {
            throw new IllegalArgumentException("Route endpoints must be in mapped clear space");
        }
        List<Point> nodes = Arrays.asList(start, goal,
                new Point(-KEEP_OUT_X - 2, -KEEP_OUT_Y - 2),
                new Point(-KEEP_OUT_X - 2, KEEP_OUT_Y + 2),
                new Point(KEEP_OUT_X + 2, -KEEP_OUT_Y - 2),
                new Point(KEEP_OUT_X + 2, KEEP_OUT_Y + 2));
        double[] distance = new double[nodes.size()];
        Arrays.fill(distance, Double.POSITIVE_INFINITY);
        distance[0] = 0;
        int[] previous = new int[nodes.size()];
        Arrays.fill(previous, -1);
        boolean[] visited = new boolean[nodes.size()];
        for (int step = 0; step < nodes.size(); step++) {
            int current = -1;
            for (int i = 0; i < nodes.size(); i++) {
                if (!visited[i] && (current < 0 || distance[i] < distance[current])) {
                    current = i;
                }
            }
            if (current < 0 || !Double.isFinite(distance[current])) {
                break;
            }
            visited[current] = true;
            for (int next = 0; next < nodes.size(); next++) {
                if (!visited[next] && segmentClear(nodes.get(current), nodes.get(next))) {
                    double candidate = distance[current]
                            + nodes.get(current).distanceTo(nodes.get(next));
                    if (candidate < distance[next]) {
                        distance[next] = candidate;
                        previous[next] = current;
                    }
                }
            }
        }
        if (!Double.isFinite(distance[1])) {
            throw new IllegalStateException("No route through the mapped clear space");
        }
        List<Point> route = new ArrayList<>();
        for (int i = 1; i != 0; i = previous[i]) {
            route.add(nodes.get(i));
        }
        Collections.reverse(route);
        return Collections.unmodifiableList(route);
    }
}
