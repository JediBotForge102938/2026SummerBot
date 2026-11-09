package org.firstinspires.ftc.teamcode.control;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.assertEquals;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.atLeastOnce;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import java.io.File;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FieldMap;
import org.firstinspires.ftc.teamcode.subsystems.RobotProfile;
import org.firstinspires.ftc.teamcode.subsystems.TrackingSubsystem;
import org.junit.Test;
import org.junit.Rule;
import org.junit.rules.TemporaryFolder;

public class LandmarkNavigationServiceTest {
    @Rule public TemporaryFolder folder = new TemporaryFolder();

    private RobotProfile verifiedTestProfile() throws Exception {
        String json = new String(Files.readAllBytes(new File(
                "../robot-configs/biobuzz-starterbot.profile.json").toPath()), StandardCharsets.UTF_8)
                .replace("\"geometryVerified\": false", "\"geometryVerified\": true")
                .replace("\"pedroTuningVerified\": false", "\"pedroTuningVerified\": true");
        Files.write(new File(folder.getRoot(), "verified.profile.json").toPath(),
                json.getBytes(StandardCharsets.UTF_8));
        return RobotProfile.load(folder.getRoot(), "verified", "starterbot");
    }

    private TrackingSubsystem validTracking() {
        TrackingSubsystem tracking = mock(TrackingSubsystem.class);
        when(tracking.hasValidMotion()).thenReturn(true);
        when(tracking.getPose()).thenReturn(new FieldMap.Pose(50, -45, 0));
        return tracking;
    }

    @Test public void physicalNavigationIsBlockedUntilBothEvidenceFlagsAreVerified() {
        RobotProfile profile = RobotProfile.load(
                new File("../robot-configs"), "biobuzz-starterbot", "starterbot");
        DriveSubsystem drive = mock(DriveSubsystem.class);
        TrackingSubsystem tracking = mock(TrackingSubsystem.class);
        LandmarkNavigationService navigation =
                new LandmarkNavigationService(drive, tracking, profile);
        LandmarkFieldMap.Destination target = new LandmarkFieldMap(profile)
                .destinations().get(0);

        assertFalse(profile.navigation.geometryVerified);
        assertFalse(profile.navigation.pedroTuningVerified);
        assertFalse(navigation.start(target, 0));
        assertEquals("Blocked", navigation.getPhase());
        assertTrue(navigation.getStatus().contains("not verified"));
        verify(drive, atLeastOnce()).driveRobotRelative(0, 0, 0);
    }

    @Test public void invalidMotionStopsAndFrozenRobotHitsProgressTimeout() throws Exception {
        DriveSubsystem drive = mock(DriveSubsystem.class);
        TrackingSubsystem tracking = validTracking();
        LandmarkNavigationService navigation =
                new LandmarkNavigationService(drive, tracking, verifiedTestProfile());
        LandmarkFieldMap.Destination home =
                LandmarkFieldMap.Destination.savedHome(new FieldMap.Pose(55, -45, 0));
        assertTrue(navigation.start(home, 0));
        when(tracking.hasValidMotion()).thenReturn(false);
        navigation.update(0.02);
        assertFalse(navigation.isActive());
        assertEquals("Failed", navigation.getPhase());
        assertEquals(0, navigation.getPowerLimit(), 0);
        verify(drive, atLeastOnce()).driveRobotRelative(0, 0, 0);

        when(tracking.hasValidMotion()).thenReturn(true);
        assertTrue(navigation.start(home, 1));
        navigation.update(1.02);
        navigation.update(5.03);
        assertFalse(navigation.isActive());
        assertTrue(navigation.getStatus().contains("no meaningful progress"));
    }

    @Test public void arrivalRequiresContinuousActualStationarityAndDoesNotResetPose() throws Exception {
        DriveSubsystem drive = mock(DriveSubsystem.class);
        TrackingSubsystem tracking = validTracking();
        RobotProfile profile = verifiedTestProfile();
        LandmarkNavigationService navigation = new LandmarkNavigationService(drive, tracking, profile);
        LandmarkFieldMap.Destination home = LandmarkFieldMap.Destination.savedHome(tracking.getPose());
        assertTrue(navigation.start(home, 0));
        navigation.update(0.1);
        navigation.update(0.5);
        assertTrue(navigation.isActive());
        when(tracking.isStationary(profile.navigation.stationarySpeedInPerSec,
                profile.navigation.stationaryTurnRateDegreesPerSec)).thenReturn(true);
        navigation.update(0.6);
        navigation.update(0.79);
        assertTrue(navigation.isActive());
        when(tracking.isStationary(profile.navigation.stationarySpeedInPerSec,
                profile.navigation.stationaryTurnRateDegreesPerSec)).thenReturn(false);
        navigation.update(0.8);
        when(tracking.isStationary(profile.navigation.stationarySpeedInPerSec,
                profile.navigation.stationaryTurnRateDegreesPerSec)).thenReturn(true);
        navigation.update(0.9);
        navigation.update(1.11);
        assertFalse(navigation.isActive());
        assertEquals("Arrived", navigation.getPhase());
        assertEquals(0, navigation.getPowerLimit(), 0);
        org.mockito.Mockito.verify(tracking, org.mockito.Mockito.never())
                .start(org.mockito.ArgumentMatchers.any());
        verify(drive, atLeastOnce()).driveRobotRelative(0, 0, 0);
    }
}
