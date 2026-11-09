package org.firstinspires.ftc.teamcode.control;

import static org.junit.Assert.*;

import java.util.List;
import org.firstinspires.ftc.teamcode.subsystems.FieldMap;
import org.firstinspires.ftc.teamcode.subsystems.RobotProfile;
import org.junit.Test;

public class LandmarkFieldMapTest {
    private final RobotProfile profile = RobotProfile.load(
            new java.io.File("../robot-configs"), "biobuzz-starterbot", "starterbot");
    private final LandmarkFieldMap map = new LandmarkFieldMap(profile);

    @Test public void menuHasTwelveAllianceAgnosticDestinationsAndSeparateObjects() {
        List<LandmarkFieldMap.Destination> destinations = map.destinations();
        assertEquals(12, destinations.size());
        assertEquals(4, destinations.stream()
                .filter(d -> d.kind == LandmarkFieldMap.Kind.FLOWER).count());
        assertEquals(2, destinations.stream()
                .filter(d -> d.kind == LandmarkFieldMap.Kind.GARDEN).count());
        assertEquals(2, destinations.stream()
                .filter(d -> d.kind == LandmarkFieldMap.Kind.PARKING).count());
        assertEquals(4, destinations.stream()
                .filter(d -> d.kind == LandmarkFieldMap.Kind.SHOOTING_START).count());

        assertEquals(-72, destinations.get(0).objectPose.x, 0);
        assertEquals(-24, destinations.get(0).objectPose.y, 0);
        assertEquals(-60, destinations.get(0).robotPose.x, 1e-9);
        assertEquals(-24, destinations.get(0).robotPose.y, 0);
        assertEquals(180, destinations.get(0).robotPose.headingDegrees, 0);
        assertEquals(3, Math.abs(destinations.get(0).objectPose.x
                - (destinations.get(0).robotPose.x
                + Math.cos(Math.toRadians(destinations.get(0).robotPose.headingDegrees)) * 9)),
                1e-9);
        assertEquals(-72, destinations.get(4).objectPose.x, 0);
        assertEquals(-72, destinations.get(4).objectPose.y, 0);
        assertNotNull(destinations.get(4).region);
        assertEquals(-60, destinations.get(4).robotPose.x, 1e-9);
        assertEquals(-61, destinations.get(4).robotPose.y, 1e-9);
        assertEquals(36, destinations.get(6).robotPose.y, 0);
        assertEquals(-36, destinations.get(7).robotPose.y, 0);
        assertEquals(-12, destinations.get(8).robotPose.x, 0);
        assertEquals(-63, destinations.get(8).robotPose.y, 0);
        assertEquals(90, destinations.get(8).robotPose.headingDegrees, 0);
        assertEquals(-12, destinations.get(9).robotPose.x, 0);
        assertEquals(63, destinations.get(9).robotPose.y, 0);
        assertEquals(-90, destinations.get(9).robotPose.headingDegrees, 0);
        assertEquals(12, destinations.get(10).robotPose.x, 0);
        assertEquals(-63, destinations.get(10).robotPose.y, 0);
        assertEquals(12, destinations.get(11).robotPose.x, 0);
        assertEquals(63, destinations.get(11).robotPose.y, 0);
        assertEquals(12, FieldMap.hiveCell(
                org.firstinspires.ftc.teamcode.BioBuzzLimelightClusterAdapter.Alliance.BLUE,
                org.firstinspires.ftc.teamcode.subsystems.ConfigurationSubsystem.StartingSpot.AUDIENCE).x, 0);
    }

    @Test public void mapsHeadingAlignedSafeRoutesIncludingWallStartDeparture() {
        FieldMap.Pose start = FieldMap.startingPose(
                org.firstinspires.ftc.teamcode.BioBuzzLimelightClusterAdapter.Alliance.RED,
                org.firstinspires.ftc.teamcode.subsystems.ConfigurationSubsystem.StartingSpot.AUDIENCE,
                18);
        assertFalse(map.isTransitClear(start));
        for (LandmarkFieldMap.Destination destination : map.destinations()) {
            List<LandmarkFieldMap.Waypoint> route;
            try {
                route = map.plan(start, destination);
            } catch (RuntimeException exception) {
                throw new AssertionError(destination.name, exception);
            }
            if (route.isEmpty()) {
                assertEquals(0, Math.hypot(start.x - destination.robotPose.x,
                        start.y - destination.robotPose.y), 1e-9);
                assertEquals(start.headingDegrees, destination.robotPose.headingDegrees, 0);
                continue;
            }
            FieldMap.Pose previous = start;
            boolean finalApproach = false;
            for (LandmarkFieldMap.Waypoint waypoint : route) {
                assertTrue(destination.name + " " + waypoint.segment + " "
                                + previous.x + "," + previous.y + " -> "
                                + waypoint.pose.x + "," + waypoint.pose.y,
                        map.segmentStillClear(previous, waypoint.pose,
                                waypoint.segment, destination));
                if (waypoint.segment == LandmarkFieldMap.Segment.FINAL_APPROACH) {
                    finalApproach = true;
                    assertEquals(destination.robotPose.x, waypoint.pose.x, 0);
                    assertEquals(destination.robotPose.y, waypoint.pose.y, 0);
                    assertEquals(destination.robotPose.headingDegrees,
                            waypoint.pose.headingDegrees, 0);
                }
                previous = waypoint.pose;
            }
            assertTrue(finalApproach);
            assertEquals(destination.robotPose.x, previous.x, 0);
            assertEquals(destination.robotPose.y, previous.y, 0);
        }
    }

    @Test public void refusesUnsafePoseAndDoesNotRouteThroughCentralFrame() {
        assertFalse(map.isTransitClear(new FieldMap.Pose(0, 0, 0)));
        LandmarkFieldMap.Destination target = map.destinations().get(6);
        List<LandmarkFieldMap.Waypoint> route = map.plan(
                new FieldMap.Pose(48, 0, 0), target);
        FieldMap.Pose previous = new FieldMap.Pose(48, 0, 0);
        for (LandmarkFieldMap.Waypoint waypoint : route) {
            assertTrue(map.segmentStillClear(previous, waypoint.pose,
                    waypoint.segment, target));
            previous = waypoint.pose;
        }

        assertThrows(IllegalStateException.class, () -> map.plan(
                new FieldMap.Pose(0, 0, 0), target));
    }

    @Test public void everyLandmarkSupportsSafeDepartureAndRepeatTripsForBothProfiles() {
        for (String robot : new String[] {"starterbot", "competitionbot"}) {
            LandmarkFieldMap field = new LandmarkFieldMap(RobotProfile.load(
                    new java.io.File("../robot-configs"), "biobuzz-" + robot, robot));
            for (LandmarkFieldMap.Destination from : field.destinations()) {
                for (LandmarkFieldMap.Destination to : field.destinations()) {
                    FieldMap.Pose previous = from.robotPose;
                    for (LandmarkFieldMap.Waypoint waypoint : field.plan(previous, to)) {
                        assertTrue(from.name + " -> " + to.name + ": " + waypoint.segment,
                                field.segmentStillClear(previous, waypoint.pose,
                                        waypoint.segment, to));
                        if (!waypoint.curveControlPoints.isEmpty()) {
                            assertTrue(field.isCurveTransitClear(waypoint.curveControlPoints));
                            assertEquals(previous.x, waypoint.curveControlPoints.get(0).x, 1e-9);
                            assertEquals(previous.y, waypoint.curveControlPoints.get(0).y, 1e-9);
                        }
                        previous = waypoint.pose;
                    }
                }
            }
        }
    }

    @Test public void detourUsesCertifiedCurvesAndUnsafeHomeCannotBypassSafety() {
        List<LandmarkFieldMap.Waypoint> route = map.plan(
                new FieldMap.Pose(48, 0, 0), map.destinations().get(6));
        assertTrue(route.stream().anyMatch(w -> !w.curveControlPoints.isEmpty()));
        FieldMap.Pose unsafe = new FieldMap.Pose(0, 0, 0);
        assertThrows(IllegalStateException.class, () -> map.plan(
                unsafe, LandmarkFieldMap.Destination.savedHome(unsafe)));
        FieldMap.Pose wallHome = new FieldMap.Pose(60, -36, 0);
        assertTrue(map.isSafeHome(wallHome));
        assertFalse(map.plan(wallHome, map.destinations().get(6)).isEmpty());
    }
}
