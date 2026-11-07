package org.firstinspires.ftc.teamcode.control;

import static org.junit.Assert.*;

import java.util.List;
import java.util.Random;

import org.firstinspires.ftc.teamcode.control.BioBuzzFieldMap.Point;
import org.junit.Test;

public class BioBuzzFieldMapTest {
    private final BioBuzzFieldMap map = new BioBuzzFieldMap();

    @Test
    public void inflatesOfficialFrameByRobotCircumradiusAndTwoInches() {
        assertEquals(24.73 + Math.hypot(9, 9) + 2, BioBuzzFieldMap.KEEP_OUT_X, 1e-9);
        assertEquals(19.475 + Math.hypot(9, 9) + 2, BioBuzzFieldMap.KEEP_OUT_Y, 1e-9);
        assertFalse(map.isClear(new Point(0, 0)));
        assertFalse(map.isClear(new Point(BioBuzzFieldMap.KEEP_OUT_X, 0)));
        assertFalse(map.isClear(new Point(60, 0)));
        assertFalse(map.isClear(new Point(Double.NaN, 0)));
        assertFalse(map.isClear(new Point(0, Double.POSITIVE_INFINITY)));
        assertTrue(map.isClear(new Point(-45, 0)));
    }

    @Test
    public void routesAcrossOppositeSidesAroundFrameRatherThanThroughIt() {
        Point start = new Point(45, 0);
        Point home = new Point(-49, 0);
        assertFalse(map.segmentClear(start, home));
        List<Point> route = map.plan(start, home);
        assertEquals(3, route.size());
        assertTrue(Math.abs(route.get(0).y) > BioBuzzFieldMap.KEEP_OUT_Y);
        checkRoute(start, route, home);
    }

    @Test
    public void keepsClearDirectRoutesAndHandlesZeroLengthRoutes() {
        Point start = new Point(-45, 10);
        Point goal = new Point(-49, 0);
        assertEquals(1, map.plan(start, goal).size());
        assertTrue(map.segmentClear(start, start));
        checkRoute(start, map.plan(start, start), start);
        assertFalse(map.segmentClear(new Point(0, 0), new Point(0, 0)));
        assertThrows(IllegalArgumentException.class,
                () -> map.plan(new Point(0, 0), goal));
    }

    @Test
    public void randomRoutesKeepEverySegmentAndSweptRobotOutsideFrameAndPerimeter() {
        Random random = new Random(2026);
        for (int i = 0; i < 300; i++) {
            Point start = clearPoint(random);
            Point goal = clearPoint(random);
            checkRoute(start, map.plan(start, goal), goal);
        }
    }

    private Point clearPoint(Random random) {
        Point p;
        do {
            p = new Point(random.nextDouble() * 100 - 50, random.nextDouble() * 100 - 50);
        } while (!map.isClear(p));
        return p;
    }

    private void checkRoute(Point start, List<Point> route, Point goal) {
        Point previous = start;
        for (Point next : route) {
            assertTrue(map.segmentClear(previous, next));
            for (int step = 0; step <= 100; step++) {
                double t = step / 100.0;
                double x = previous.x + (next.x - previous.x) * t;
                double y = previous.y + (next.y - previous.y) * t;
                double dx = Math.max(0, Math.abs(x) - BioBuzzFieldMap.FRAME_HALF_WIDTH);
                double dy = Math.max(0, Math.abs(y) - BioBuzzFieldMap.FRAME_HALF_DEPTH);
                assertTrue(Math.hypot(dx, dy)
                        >= BioBuzzFieldMap.ROBOT_RADIUS + BioBuzzFieldMap.CLEARANCE - 1e-9);
                assertTrue(Math.abs(x) + BioBuzzFieldMap.ROBOT_RADIUS
                        + BioBuzzFieldMap.CLEARANCE + BioBuzzFieldMap.PERIMETER_STRIP <= 72 + 1e-9);
                assertTrue(Math.abs(y) + BioBuzzFieldMap.ROBOT_RADIUS
                        + BioBuzzFieldMap.CLEARANCE + BioBuzzFieldMap.PERIMETER_STRIP <= 72 + 1e-9);
            }
            previous = next;
        }
        assertEquals(0, previous.distanceTo(goal), 1e-9);
    }
}
