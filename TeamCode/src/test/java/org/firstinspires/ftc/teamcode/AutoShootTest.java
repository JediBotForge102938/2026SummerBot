package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.*;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.ArrayList;
import java.io.File;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.control.DriveModePreferences;
import org.firstinspires.ftc.teamcode.control.LandmarkFieldMap;
import org.firstinspires.ftc.teamcode.control.LandmarkNavigationService;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FieldMap;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherFeeder;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotProfile;
import org.firstinspires.ftc.teamcode.subsystems.TrackingSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WindmillSubsystem;
import org.junit.Test;
import org.mockito.MockedConstruction;
import org.mockito.MockedStatic;

public class AutoShootTest {
    @Test public void registeredAndSafeToStopBeforeInit() {
        assertEquals("Auto Shoot", AutoShoot.class.getAnnotation(Autonomous.class).name());
        new AutoShoot().stop();
    }

    @Test public void shootsPreloadsCollectsOwnGardenAndRechecksHiveForEveryStart() {
        for (boolean red : new boolean[] {true, false}) {
            for (boolean scoring : new boolean[] {true, false}) {
                try (Harness h = new Harness(red, scoring)) {
                    h.start();
                    h.arriveAndShoot(0.1);
                    h.loop(0.5 + AutoShoot.BATCH_FEED_SECONDS);
                    h.loop(0.6 + AutoShoot.BATCH_FEED_SECONDS + AutoShoot.HIVE_SETTLE_SECONDS);
                    assertEquals(LandmarkFieldMap.Kind.GARDEN, h.target.get().kind);
                    assertEquals(red ? "Red garden collection" : "Blue garden collection",
                            h.target.get().name);
                    verify(h.intake, atLeastOnce()).setPower(1);
                    h.loop(4);
                    h.loop(4 + AutoShoot.COLLECTION_SECONDS);
                    assertEquals(LandmarkFieldMap.Kind.GARDEN, h.target.get().kind);
                    h.loop(6.1);
                    h.loop(6.8);
                    assertEquals(LandmarkFieldMap.Kind.HIVE_SHOOTING, h.target.get().kind);
                    assertEquals(scoring, h.target.get().objectPose.y > 0);
                    h.arriveAndShoot(6.9);
                    h.loop(7.5 + AutoShoot.BATCH_FEED_SECONDS);
                    assertEquals(LandmarkFieldMap.Kind.PARKING, h.target.get().kind);
                    h.loop(10.2);
                    h.assertStopped();
                    verify(h.op.telemetry).addLine("Parked: Completed timed garden batch");
                }
            }
        }
    }

    @Test public void visibleScorableOtherHiveSelectsItsPositionButLossThenParks() {
        try (Harness h = new Harness(true, false)) {
            h.start();
            h.arriveAndShoot(0.1);
            h.loop(3.2);
            h.showCell(false, false);
            h.loop(3.9);
            assertEquals(LandmarkFieldMap.Kind.GARDEN, h.target.get().kind);
            h.loop(4);
            h.loop(6.1);
            assertEquals(LandmarkFieldMap.Kind.GARDEN, h.target.get().kind);
            h.showCell(true, true);
            h.loop(6.2);
            clearInvocations(h.feeder);
            h.loop(6.9);
            verify(h.feeder, never()).update(true);
            assertTrue(h.target.get().objectPose.y > 0);
            h.showCell(true, false);
            h.loop(7.0);
            h.loop(7.1);
            assertEquals(LandmarkFieldMap.Kind.PARKING, h.target.get().kind);
            h.loop(7.2);
            h.assertStopped();
            verify(h.op.telemetry).addLine("Parked: Selected hive missing or unscorable");
        }
    }

    @Test public void preloadsDoNotRequireAprilTagsAndAlwaysProceedToGardenAfterFeeding() {
        for (int invalid = 0; invalid < 5; invalid++) {
            try (Harness h = new Harness(true, false)) {
                h.start();
                h.loop(0.1);
                switch (invalid) {
                    case 0: when(h.result.getStaleness()).thenReturn(251L); break;
                    case 1: h.tags.clear(); break;
                    case 2: h.tags.remove(1); break;
                    case 3: h.addTags(38, -10, 0); break;
                    case 4: h.addTags(34, -10, 30); break;
                    default: fail();
                }
                h.loop(0.2);
                h.loop(0.6);
                verify(h.feeder).update(true);
                verify(h.camera, never()).getLatestResult();
                h.loop(3.3);
                assertEquals(LandmarkFieldMap.Kind.GARDEN, h.target.get().kind);
                verify(h.intake).setPower(1);
            }
        }
    }

    @Test public void preloadVisionLossDoesNotStopFeedButLauncherReadinessLossDoes() {
        for (boolean loseVision : new boolean[] {true, false}) {
            try (Harness h = new Harness(true, false)) {
                h.start();
                h.arriveAndShoot(0.1);
                clearInvocations(h.feeder, h.intake);
                if (loseVision) h.tags.clear();
                else when(h.launcher.update(true)).thenReturn(false);
                h.loop(0.6);
                verify(h.feeder).update(loseVision);
                verify(h.intake).setPower(loseVision ? 0.5 : 0);
            }
        }
    }

    @Test public void preloadHiveOrientationDoesNotPreventGardenCollection() {
        try (Harness h = new Harness(true, false)) {
            h.start();
            h.arriveAndShoot(0.1);
            clearInvocations(h.feeder);
            h.showCell(false, false);
            h.loop(1.0);
            verify(h.feeder).update(true);
            verify(h.camera, never()).getLatestResult();
            h.loop(3.2);
            assertEquals(LandmarkFieldMap.Kind.GARDEN, h.target.get().kind);
        }
    }

    @Test public void competitionHardwareUsesSingleLauncherAndIntakeWithoutStarterServos() {
        RobotProfile competition = RobotProfile.load(new File("../robot-configs"),
                "biobuzz-competitionbot", "competitionbot");
        try (Harness h = new Harness(false, true, competition)) {
            assertTrue(h.feeders.constructed().isEmpty());
            assertEquals(1, h.gates.constructed().size());
            verify(h.gates.constructed().get(0)).init(h.op.hardwareMap);
            assertEquals(Boolean.FALSE, h.intakeArguments.get(0));
            assertSame(h.profile, h.intakeArguments.get(1));
            List<?> configs = (List<?>) h.launcherArguments.get(0);
            assertSame(competition.launcherMotors, configs);
            assertSame(competition.launcherPidf, h.launcherArguments.get(1));
            assertEquals(competition.launcherFeedVelocityMargin, h.launcherArguments.get(2));
            assertEquals(1, configs.size());
            LauncherSubsystem.MotorConfig launcher = (LauncherSubsystem.MotorConfig) configs.get(0);
            assertEquals("launcher", launcher.name);
            assertEquals(competition.launcherMotors.get(0).direction, launcher.direction);
            h.start();
            h.arriveAndShoot(0.1);
            verify(h.feeder).update(true);
            when(h.launcher.update(true)).thenReturn(false);
            h.loop(0.49);
            verify(h.feeder, atLeastOnce()).update(false);
            verify(h.intake).setPower(0.5);
            h.op.gamepad1.b = true;
            h.loop(0.5);
            h.assertStopped();
        }
    }

    @Test public void feedingWaitsForLauncherReadinessAndStableAim() {
        try (Harness h = new Harness(true, false)) {
            when(h.launcher.update(true)).thenReturn(false);
            h.start();
            h.loop(0.1);
            h.loop(0.2);
            h.loop(0.6);
            verify(h.feeder, never()).update(true);
            when(h.launcher.update(true)).thenReturn(true);
            when(h.tracking.isStationary(anyDouble(), anyDouble())).thenReturn(false);
            h.loop(0.7);
            verify(h.feeder, never()).update(true);
            when(h.tracking.isStationary(anyDouble(), anyDouble())).thenReturn(true);
            h.loop(0.8);
            h.loop(0.9);
            verify(h.feeder, never()).update(true);
            h.loop(1.1);
            verify(h.feeder).update(true);
        }
    }

    @Test public void missingPostShotTagsDoNotBlockGardenAndAreCheckedOnlyAfterCollection() {
        try (Harness h = new Harness(true, false)) {
            h.start();
            h.arriveAndShoot(0.1);
            h.tags.clear();
            clearInvocations(h.camera);
            h.loop(3.2);
            assertEquals(LandmarkFieldMap.Kind.GARDEN, h.target.get().kind);
            h.loop(3.9);
            assertEquals(LandmarkFieldMap.Kind.GARDEN, h.target.get().kind);
            verify(h.intake, atLeastOnce()).setPower(1);
            verify(h.camera, never()).getLatestResult();
            h.loop(6.0);
            assertEquals(LandmarkFieldMap.Kind.GARDEN, h.target.get().kind);
            h.loop(6.1);
            verify(h.camera, never()).getLatestResult();
            clearInvocations(h.feeder);
            h.loop(6.8);
            verify(h.feeder, never()).update(true);
            assertEquals(LandmarkFieldMap.Kind.PARKING, h.target.get().kind);
            h.loop(6.9);
            h.assertStopped();
            verify(h.op.telemetry).addLine(
                    "Parked: No visible scorable alliance hive after collection");
        }
    }

    @Test public void invalidOrUnscorablePostCollectionTagsParkWithoutTravellingToHive() {
        for (int invalid = 0; invalid < 5; invalid++) {
            try (Harness h = new Harness(true, false)) {
                h.start();
                h.arriveAndShoot(0.1);
                h.loop(3.2);
                h.loop(3.3);
                h.loop(5.4);
                clearInvocations(h.navigation, h.feeder);
                switch (invalid) {
                    case 0: h.showCell(false, false); break;
                    case 1: h.tags.clear(); break;
                    case 2: when(h.result.getStaleness()).thenReturn(251L); break;
                    case 3: h.tags.remove(1); break;
                    case 4: h.addTags(38, -10, 0); break;
                    default: fail();
                }
                h.loop(6.1);
                verify(h.navigation).start(argThat(target ->
                        target.kind == LandmarkFieldMap.Kind.PARKING), anyDouble());
                verify(h.navigation, times(1)).start(any(), anyDouble());
                verify(h.feeder, never()).update(true);
                h.loop(6.2);
                h.assertStopped();
            }
        }
    }

    @Test public void failedParkingNavigationStopsInsteadOfReportingParked() {
        try (Harness h = new Harness(true, false)) {
            h.start();
            h.arriveAndShoot(0.1);
            h.loop(3.2);
            h.loop(3.3);
            h.loop(5.4);
            h.tags.clear();
            h.loop(6.1);
            when(h.navigation.getPhase()).thenReturn("Failed");
            when(h.navigation.getStatus()).thenReturn("Parking route failed");
            h.loop(6.2);
            h.assertStopped();
            verify(h.op.telemetry).addLine("Parking route failed");
            verify(h.op.telemetry, never()).addLine(startsWith("Parked:"));
        }
    }

    @Test public void profileNavigationTrackingDriverAndDeadlineFailuresStopEverything() {
        for (int failure = 0; failure < 7; failure++) {
            try (Harness h = new Harness(true, false)) {
                if (failure == 0) doReturn(false).when(h.profile).isAutonomousReady();
                if (failure == 1) when(h.navigation.start(any(), anyDouble())).thenReturn(false);
                h.start();
                if (failure == 2) when(h.navigation.getPhase()).thenReturn("Failed");
                if (failure == 3) when(h.tracking.hasValidMotion()).thenReturn(false);
                if (failure == 4) h.op.gamepad1.b = true;
                if (failure == 5) h.op.gamepad1.y = true;
                h.loop(failure == 6 ? AutoShoot.AUTO_SECONDS : 0.1);
                h.assertStopped();
                verify(h.feeder, never()).update(true);
                verify(h.intake, never()).setPower(1);
                h.op.stop();
                verify(h.camera).stop();
            }
        }
    }

    @Test public void collectionCannotStartWhileNavigationIsStillActive() {
        try (Harness h = new Harness(true, false)) {
            h.start();
            when(h.navigation.isActive()).thenReturn(true);
            h.loop(1);
            h.loop(2);
            verify(h.navigation, times(2)).update(anyDouble());
            verify(h.feeder, never()).update(true);
            verify(h.intake, never()).setPower(1);
        }
    }

    @Test public void wallPressRunsOnlyDuringCollectionAndStopsBeforeTagCheck() {
        for (boolean red : new boolean[] {true, false}) {
            try (Harness h = new Harness(red, false)) {
                h.start();
                h.arriveAndShoot(0.1);
                h.loop(3.2);
                h.loop(3.3);
                DriveSubsystem drive = h.drives.constructed().get(0);
                verify(drive).driveRobotRelative(AutoShoot.GARDEN_PRESS_POWER, 0, 0);
                clearInvocations(drive);
                h.loop(4);
                verify(drive).driveRobotRelative(AutoShoot.GARDEN_PRESS_POWER, 0, 0);
                clearInvocations(drive);
                h.loop(5.4);
                verify(drive).driveRobotRelative(0, 0, 0);
                verify(drive, never()).driveRobotRelative(AutoShoot.GARDEN_PRESS_POWER, 0, 0);
                verify(h.intake, atLeastOnce()).setPower(0);
            }
        }
    }

    @Test public void wallPressStopsOnMisalignmentTrackingLossOrDriverStop() {
        for (int failure = 0; failure < 3; failure++) {
            try (Harness h = new Harness(true, false)) {
                h.start();
                h.arriveAndShoot(0.1);
                h.loop(3.2);
                h.loop(3.3);
                if (failure == 0) {
                    FieldMap.Pose pose = h.target.get().robotPose;
                    when(h.tracking.getPose()).thenReturn(new FieldMap.Pose(pose.x, pose.y, 0));
                } else if (failure == 1) {
                    when(h.tracking.hasValidMotion()).thenReturn(false);
                } else {
                    h.op.gamepad1.b = true;
                }
                clearInvocations(h.drives.constructed().get(0));
                h.loop(3.5);
                verify(h.drives.constructed().get(0), never())
                        .driveRobotRelative(AutoShoot.GARDEN_PRESS_POWER, 0, 0);
                h.assertStopped();
            }
        }
    }

    @Test public void wallPressStopsIfFootprintCrossesSideWall() {
        for (boolean red : new boolean[] {true, false}) {
            try (Harness h = new Harness(red, false)) {
                h.start();
                h.arriveAndShoot(0.1);
                h.loop(3.2);
                h.loop(3.3);
                FieldMap.Pose pose = h.target.get().robotPose;
                when(h.tracking.getPose()).thenReturn(new FieldMap.Pose(
                        pose.x + (red ? -0.75 : 0.75), pose.y, pose.headingDegrees));
                clearInvocations(h.drives.constructed().get(0));
                h.loop(3.5);
                verify(h.drives.constructed().get(0), never())
                        .driveRobotRelative(AutoShoot.GARDEN_PRESS_POWER, 0, 0);
                h.assertStopped();
            }
        }
    }

    private static final class Harness implements AutoCloseable {
        final RobotProfile profile;
        final AtomicReference<LandmarkFieldMap.Destination> target = new AtomicReference<>();
        final MockedStatic<RobotProfile> profiles = mockStatic(RobotProfile.class);
        final MockedConstruction<DriveSubsystem> drives = mockConstruction(DriveSubsystem.class);
        final MockedConstruction<TrackingSubsystem> trackers =
                mockConstruction(TrackingSubsystem.class, (mock, context) -> {
                    when(mock.hasValidMotion()).thenReturn(true);
                    when(mock.isStationary(anyDouble(), anyDouble())).thenReturn(true);
                    when(mock.getPose()).thenAnswer(invocation -> target.get().robotPose);
                });
        final List<Object> intakeArguments = new ArrayList<>();
        final List<Object> launcherArguments = new ArrayList<>();
        final MockedConstruction<IntakeSubsystem> intakes =
                mockConstruction(IntakeSubsystem.class, (mock, context) ->
                        intakeArguments.addAll(context.arguments()));
        final MockedConstruction<LauncherSubsystem> launchers =
                mockConstruction(LauncherSubsystem.class, (mock, context) -> {
                    launcherArguments.addAll(context.arguments());
                    when(mock.update(true)).thenReturn(true);
                });
        final MockedConstruction<WindmillSubsystem> feeders = mockConstruction(WindmillSubsystem.class);
        final MockedConstruction<GateSubsystem> gates;
        final MockedConstruction<LandmarkNavigationService> navigators =
                mockConstruction(LandmarkNavigationService.class, (mock, context) -> {
                    when(mock.start(any(), anyDouble())).thenAnswer(invocation -> {
                        target.set(invocation.getArgument(0));
                        return true;
                    });
                    when(mock.getPhase()).thenReturn("Arrived");
                });
        final MockedConstruction<DriveModePreferences> prefs;
        final Limelight3A camera = mock(Limelight3A.class);
        final LLResult result = mock(LLResult.class);
        final List<LLResultTypes.FiducialResult> tags = new ArrayList<>();
        final AutoShoot op;
        final LauncherFeeder feeder;
        final IntakeSubsystem intake;
        final LauncherSubsystem launcher;
        final TrackingSubsystem tracking;
        final LandmarkNavigationService navigation;
        final boolean red;

        Harness(boolean red, boolean scoring) {
            this(red, scoring, TestProfiles.starter());
        }

        Harness(boolean red, boolean scoring, RobotProfile selectedProfile) {
            this.red = red;
            profile = spy(selectedProfile);
            gates = mockConstruction(GateSubsystem.class,
                    (mock, context) -> assertSame(profile.launcherGate, context.arguments().get(0)));
            prefs = mockConstruction(DriveModePreferences.class, (mock, context) -> {
                when(mock.getConfigurationRedAlliance()).thenReturn(red);
                when(mock.isScoringStart()).thenReturn(scoring);
            });
            doReturn(true).when(profile).isAutonomousReady();
            profiles.when(() -> RobotProfile.loadActive(null)).thenReturn(profile);
            when(result.isValid()).thenReturn(true);
            when(result.getFiducialResults()).thenReturn(tags);
            when(camera.getLatestResult()).thenReturn(result);
            op = spy(new AutoShoot());
            op.hardwareMap = mock(HardwareMap.class);
            when(op.hardwareMap.get(Limelight3A.class, "limelight")).thenReturn(camera);
            op.gamepad1 = mock(Gamepad.class);
            op.telemetry = mock(Telemetry.class);
            op.init();
            op.init_loop();
            feeder = feeders.constructed().isEmpty()
                    ? gates.constructed().get(0) : feeders.constructed().get(0);
            intake = intakes.constructed().get(0);
            launcher = launchers.constructed().get(0);
            tracking = trackers.constructed().get(0);
            navigation = navigators.constructed().get(0);
            showCell(scoring, true);
        }

        void start() {
            doReturn(0.0).when(op).getRuntime();
            op.start();
        }

        void loop(double now) {
            doReturn(now).when(op).getRuntime();
            op.loop();
        }

        void arriveAndShoot(double arrival) {
            loop(arrival);
            loop(arrival + 0.05);
            loop(arrival + 0.05 + AutoShoot.AIM_DWELL_SECONDS + 0.01);
            if (feeder != null) verify(feeder, atLeastOnce()).update(true);
            else verify(intake, atLeastOnce()).setPower(0.5);
        }

        void showCell(boolean scoring, boolean scorable) {
            addTags(red ? scoring ? 30 : 34 : scoring ? 42 : 38,
                    scorable ? -10 : 10, 0);
        }

        void addTags(int firstId, double roll, double x) {
            tags.clear();
            for (int i = 0; i < 2; i++) {
                LLResultTypes.FiducialResult tag = mock(LLResultTypes.FiducialResult.class);
                when(tag.getFiducialId()).thenReturn(firstId + i);
                when(tag.getTargetPoseCameraSpace()).thenReturn(new Pose3D(
                        new Position(DistanceUnit.INCH,
                                (profile.limelightInverted ? -1 : 1)
                                        * (x + (i == 0 ? -6.5 : -2.75)), 0, 100, 0),
                        new YawPitchRollAngles(
                                org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES,
                                profile.limelightInverted ? 180 : 0, 0,
                                roll - profile.limelightPitchUpDegrees, 0)));
                tags.add(tag);
            }
        }

        void assertStopped() {
            verify(launcher, atLeastOnce()).update(false);
            if (feeder != null) verify(feeder, atLeastOnce()).update(false);
            verify(intake, atLeastOnce()).setPower(0);
            verify(drives.constructed().get(0), atLeastOnce()).driveRobotRelative(0, 0, 0);
            if (feeder != null) clearInvocations(feeder);
            clearInvocations(intake, launcher, navigation);
            loop(29.9);
            if (feeder != null) verify(feeder, never()).update(true);
            verify(launcher, never()).update(true);
            verify(intake, never()).setPower(1);
            verifyNoInteractions(navigation);
        }

        @Override public void close() {
            op.stop();
            prefs.close();
            navigators.close();
            gates.close();
            feeders.close();
            launchers.close();
            intakes.close();
            trackers.close();
            drives.close();
            profiles.close();
        }
    }
}
