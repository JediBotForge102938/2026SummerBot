package org.firstinspires.ftc.teamcode.control;

import static org.junit.Assert.*;

import java.io.File;
import org.firstinspires.ftc.teamcode.BioBuzzLimelightClusterAdapter.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.ConfigurationSubsystem.StartingSpot;
import org.firstinspires.ftc.teamcode.subsystems.FieldMap;
import org.firstinspires.ftc.teamcode.subsystems.RobotProfile;
import org.junit.Test;

public class AutoShootFieldMapTest {
    @Test public void shootingAndGardenRoutesAreClearForBothRobotsAlliancesAndStarts() {
        for (String robot : new String[] {"starterbot", "competitionbot"}) {
            RobotProfile profile = RobotProfile.load(new File("../robot-configs"),
                    "biobuzz-" + robot, robot);
            LandmarkFieldMap map = new LandmarkFieldMap(profile);
            for (Alliance alliance : new Alliance[] {Alliance.RED, Alliance.BLUE}) {
                LandmarkFieldMap.Destination garden = map.gardenCollectionDestination(alliance);
                assertEquals(LandmarkFieldMap.Kind.GARDEN, garden.kind);
                assertEquals(alliance == Alliance.RED ? -70 : 70, garden.objectPose.y, 0);
                assertEquals(alliance == Alliance.RED ? -90 : 90,
                        garden.robotPose.headingDegrees, 0);
                double front = Math.toRadians(garden.robotPose.headingDegrees);
                assertEquals(profile.navigation.flowerStandoffIn,
                        Math.hypot(garden.objectPose.x - garden.robotPose.x,
                                garden.objectPose.y - garden.robotPose.y)
                                - profile.chassisLengthCm / 2 / 2.54, 1e-9);
                assertEquals(Math.signum(garden.objectPose.y - garden.robotPose.y),
                        Math.sin(front), 1e-9);
                for (StartingSpot side : StartingSpot.values()) {
                    LandmarkFieldMap.Destination shooting = map.hiveShootingDestination(alliance, side);
                    assertTrue(map.isTransitClear(shooting.robotPose));
                    FieldMap.Pose cell = FieldMap.hiveCell(alliance, side);
                    double rearHeading = Math.toRadians(shooting.robotPose.headingDegrees + 180);
                    double bearing = Math.atan2(cell.y - shooting.robotPose.y,
                            cell.x - shooting.robotPose.x);
                    assertEquals(1, Math.cos(bearing - rearHeading), 1e-9);
                    assertEquals(cell.x, shooting.robotPose.x, 0);
                    assertEquals(side == StartingSpot.AUDIENCE ? -90 : 90,
                            shooting.robotPose.headingDegrees, 0);
                    assertRoute(map, FieldMap.startingPose(alliance, side,
                            profile.chassisLengthCm / 2.54), shooting);
                    assertRoute(map, shooting.robotPose, garden);
                    assertRoute(map, garden.robotPose, shooting);
                    FieldMap.Pose againstWall = new FieldMap.Pose(garden.robotPose.x,
                            Math.copySign(72 - profile.chassisLengthCm / 2 / 2.54,
                                    garden.robotPose.y), garden.robotPose.headingDegrees);
                    assertRoute(map, againstWall, shooting);
                    assertRoute(map, againstWall, map.parkingDestination(alliance, side));
                    assertRoute(map, shooting.robotPose,
                            map.hiveShootingDestination(alliance, side == StartingSpot.AUDIENCE
                                    ? StartingSpot.SCORING : StartingSpot.AUDIENCE));
                }
            }
        }
    }

    @Test public void invalidAllianceAndSideAreRejected() {
        LandmarkFieldMap map = new LandmarkFieldMap(RobotProfile.load(new File("../robot-configs"),
                "biobuzz-starterbot", "starterbot"));
        assertThrows(IllegalArgumentException.class, () -> map.gardenDestination(Alliance.BOTH));
        assertThrows(IllegalArgumentException.class,
                () -> map.hiveShootingDestination(Alliance.RED, null));
    }

    private static void assertRoute(LandmarkFieldMap map, FieldMap.Pose start,
            LandmarkFieldMap.Destination target) {
        FieldMap.Pose previous = start;
        assertFalse(map.plan(start, target).isEmpty());
        for (LandmarkFieldMap.Waypoint waypoint : map.plan(start, target)) {
            assertTrue(target.name + " " + waypoint.segment,
                    map.segmentStillClear(previous, waypoint, target));
            previous = waypoint.pose;
        }
        assertEquals(target.robotPose.x, previous.x, 0);
        assertEquals(target.robotPose.y, previous.y, 0);
    }
}
