package org.firstinspires.ftc.teamcode.control;

import static org.junit.Assert.*;

import java.util.List;
import org.firstinspires.ftc.teamcode.BioBuzzLimelightClusterAdapter.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.ConfigurationSubsystem.StartingSpot;
import org.firstinspires.ftc.teamcode.subsystems.FieldMap;
import org.junit.Test;

public class BioBuzzFieldMapTest {
    @Test public void allLandmarksAreSelectableAndRoutableFromStandardStarts() {
        String[] names = {"Red wall flower", "Blue wall flower", "Red audience flower",
                "Blue scoring flower", "Red garden", "Blue garden", "Red parking",
                "Blue parking", "Red audience shooting start", "Red scoring shooting start",
                "Blue audience shooting start", "Blue scoring shooting start"};
        for (double[] size : new double[][] {{18, 18}, {45 / 2.54, 39 / 2.54}}) {
            BioBuzzFieldMap map = new BioBuzzFieldMap(size[0], size[1]);
            List<BioBuzzFieldMap.Destination> destinations = map.destinations();
            assertEquals(names.length, destinations.size());
            for (int i = 0; i < names.length; i++) {
                assertEquals(names[i], destinations.get(i).name);
                for (Alliance alliance : new Alliance[] {Alliance.RED, Alliance.BLUE}) {
                    for (StartingSpot side : StartingSpot.values()) {
                        FieldMap.Pose start = map.startingPose(alliance, side);
                        FieldMap.Pose goal = destinations.get(i).pose;
                        List<FieldMap.Pose> route = map.plan(start, goal);
                        if (!route.isEmpty()) checkRoute(start, route, goal, size[0], size[1]);
                    }
                }
            }
        }
    }

    @Test public void routesAllStartingPosesAroundFrameAndFlowersForBothProfiles() {
        for (double[] size : new double[][] {{18, 18}, {45 / 2.54, 39 / 2.54}}) {
            BioBuzzFieldMap map = new BioBuzzFieldMap(size[0], size[1]);
            for (Alliance a : new Alliance[] {Alliance.RED, Alliance.BLUE}) {
                for (StartingSpot side : StartingSpot.values()) {
                    FieldMap.Pose start = map.startingPose(a, side);
                    for (Alliance b : new Alliance[] {Alliance.RED, Alliance.BLUE}) {
                        for (StartingSpot other : StartingSpot.values()) {
                            FieldMap.Pose goal = map.startingPose(b, other);
                            List<FieldMap.Pose> route = map.plan(start, goal);
                            if (a == b && side == other) {
                                assertTrue(route.isEmpty());
                            } else {
                                assertFalse(route.isEmpty());
                                checkRoute(start, route, goal, size[0], size[1]);
                            }
                        }
                    }
                }
            }
        }
    }

    @Test public void startingPosesLeaveRoomForFlowers() {
        BioBuzzFieldMap starter = new BioBuzzFieldMap(18, 18);
        assertEquals(10.65, starter.startingPose(Alliance.BLUE, StartingSpot.AUDIENCE).x, 1e-8);
        assertEquals(-10.65, starter.startingPose(Alliance.RED, StartingSpot.SCORING).x, 1e-8);
        assertEquals(-12, starter.startingPose(Alliance.RED, StartingSpot.AUDIENCE).x, 1e-8);
        BioBuzzFieldMap competition = new BioBuzzFieldMap(45 / 2.54, 39 / 2.54);
        assertEquals(24 - 3.35 - (39 / 2.54) / 2 - 1,
                competition.startingPose(Alliance.BLUE, StartingSpot.AUDIENCE).x, 1e-8);
    }

    @Test public void landmarkTargetsPreserveOffsetsExceptFlowerClearance() {
        List<BioBuzzFieldMap.Destination> goals = new BioBuzzFieldMap(18, 18).destinations();
        assertPose(goals.get(0).pose, -55.43, -24, 180);
        assertPose(goals.get(1).pose, 55.43, 24, 0);
        assertPose(goals.get(2).pose, 24, -55.43, -90);
        assertPose(goals.get(3).pose, -24, 55.43, 90);
        assertPose(goals.get(4).pose, -62.5, -58, -90);
        assertPose(goals.get(5).pose, 62.5, 58, 90);
        assertPose(goals.get(6).pose, -60, 36, 180);
        assertPose(goals.get(7).pose, 60, -36, 0);
    }

    private static void assertPose(FieldMap.Pose pose, double x, double y, double heading) {
        assertEquals(x, pose.x, 1e-8);
        assertEquals(y, pose.y, 1e-8);
        assertEquals(heading, pose.headingDegrees, 1e-8);
    }

    @Test public void rejectsFrameFlowerAndOutOfBoundsEndpoints() {
        BioBuzzFieldMap map = new BioBuzzFieldMap(18, 18);
        FieldMap.Pose valid = new FieldMap.Pose(0, -50, 90);
        assertThrows(IllegalArgumentException.class,
                () -> map.plan(valid, new FieldMap.Pose(0, 0, 0)));
        assertThrows(IllegalArgumentException.class,
                () -> map.plan(valid, new FieldMap.Pose(24, -63, 90)));
        assertThrows(IllegalArgumentException.class,
                () -> map.plan(valid, new FieldMap.Pose(70, 0, 0)));
        assertThrows(IllegalArgumentException.class,
                () -> new BioBuzzFieldMap(0, 18));
    }

    private static void checkRoute(FieldMap.Pose start, List<FieldMap.Pose> route,
            FieldMap.Pose goal, double length, double width) {
        double radius = Math.hypot(length / 2, width / 2);
        FieldMap.Pose previous = start;
        List<FieldMap.Region> flowers = BioBuzzFieldMap.flowerObstacles();
        for (int index = 0; index < route.size(); index++) {
            FieldMap.Pose next = route.get(index);
            for (int step = 0; step <= 100; step++) {
                double t = step / 100.0;
                double x = previous.x + (next.x - previous.x) * t;
                double y = previous.y + (next.y - previous.y) * t;
                double dx = Math.max(0, Math.abs(x) - 49.46 / 2);
                double dy = Math.max(0, Math.abs(y) - 38.95 / 2);
                if (index > 0 && index < route.size() - 1) {
                    assertTrue("frame clearance", Math.hypot(dx, dy) >= radius + 1 - 1e-8);
                    for (FieldMap.Region flower : flowers) {
                        double fx = Math.max(0, Math.max(flower.minX - x, x - flower.maxX));
                        double fy = Math.max(0, Math.max(flower.minY - y, y - flower.maxY));
                        assertTrue("flower clearance", Math.hypot(fx, fy) >= radius + 1 - 1e-8);
                    }
                }
                double h = Math.toRadians(previous.headingDegrees
                        + Math.IEEEremainder(next.headingDegrees - previous.headingDegrees, 360) * t);
                double xExtent = Math.abs(Math.cos(h)) * length / 2 + Math.abs(Math.sin(h)) * width / 2;
                double yExtent = Math.abs(Math.sin(h)) * length / 2 + Math.abs(Math.cos(h)) * width / 2;
                assertTrue("wall x", Math.abs(x) + xExtent <= 72 + 1e-8);
                assertTrue("wall y", Math.abs(y) + yExtent <= 72 + 1e-8);
                assertTrue("frame footprint", Math.abs(x) > 49.46 / 2 + xExtent
                        || Math.abs(y) > 38.95 / 2 + yExtent);
                for (FieldMap.Region flower : flowers) {
                    assertTrue("flower footprint", x + xExtent < flower.minX
                            || x - xExtent > flower.maxX || y + yExtent < flower.minY
                            || y - yExtent > flower.maxY);
                }
            }
            previous = next;
        }
        assertEquals(goal.x, previous.x, 1e-8);
        assertEquals(goal.y, previous.y, 1e-8);
        assertEquals(goal.headingDegrees, previous.headingDegrees, 1e-8);
    }
}
