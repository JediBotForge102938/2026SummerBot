package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.*;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.drivetrain.Drivetrain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.io.File;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control.ProfiledPedroFollower;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FieldMap;
import org.firstinspires.ftc.teamcode.subsystems.RobotProfile;
import org.firstinspires.ftc.teamcode.subsystems.RobotProfileTuningStore;
import org.firstinspires.ftc.teamcode.subsystems.TrackingSubsystem;
import org.junit.Test;
import org.mockito.MockedConstruction;
import org.mockito.MockedStatic;

public class PedroChassisTuningTeleOpTest {
    private static final class Fixture implements AutoCloseable {
        final RobotProfile profile = RobotProfile.load(new File("../robot-configs"),
                "biobuzz-competitionbot", "competitionbot");
        final MockedStatic<RobotProfile> profiles = mockStatic(RobotProfile.class);
        final MockedConstruction<DriveSubsystem> drives = mockConstruction(DriveSubsystem.class);
        final MockedConstruction<TrackingSubsystem> trackers = mockConstruction(TrackingSubsystem.class);
        final MockedStatic<ProfiledPedroFollower> factory = mockStatic(ProfiledPedroFollower.class);
        final MockedConstruction<RobotProfileTuningStore> stores = mockConstruction(RobotProfileTuningStore.class);
        final Follower follower = mock(Follower.class);
        final PedroChassisTuningTeleOp op = spy(new PedroChassisTuningTeleOp());
        final TrackingSubsystem tracking;
        final DriveSubsystem drive;

        Fixture() {
            profiles.when(() -> RobotProfile.loadActive(null)).thenReturn(profile);
            factory.when(() -> ProfiledPedroFollower.create(any(), any(), eq(profile), eq(1.0)))
                    .thenReturn(follower);
            FollowerConstants constants = new FollowerConstants();
            constants.defaults();
            when(follower.getConstants()).thenReturn(constants);
            when(follower.getDrivetrain()).thenReturn(mock(Drivetrain.class));
            op.hardwareMap = mock(HardwareMap.class);
            op.gamepad1 = new Gamepad();
            op.telemetry = mock(Telemetry.class);
            time(0);
            op.init();
            drive = drives.constructed().get(0);
            tracking = trackers.constructed().get(0);
            when(tracking.hasValidMotion()).thenReturn(true);
            when(tracking.getPose()).thenReturn(new FieldMap.Pose(0, 0, 0));
            when(tracking.isStationary(anyDouble(), anyDouble())).thenReturn(true);
        }

        void time(double time) { doReturn(time).when(op).getRuntime(); }
        void mode(int index) {
            for (int i = 0; i < index; i++) {
                op.gamepad1.dpad_up = true;
                op.init_loop();
                op.gamepad1.dpad_up = false;
                op.init_loop();
            }
        }
        void run() {
            op.start();
            op.gamepad1.a = true;
            op.loop();
            op.gamepad1.a = false;
        }
        @Override public void close() {
            stores.close();
            factory.close();
            trackers.close();
            drives.close();
            profiles.close();
        }
    }

    @Test public void registrationAndStopBeforeInit() {
        assertEquals("Pedro Chassis Tuning",
                PedroChassisTuningTeleOp.class.getAnnotation(TeleOp.class).name());
        new PedroChassisTuningTeleOp().stop();
    }

    @Test public void initAndHeldAAtStartNeverLaunchMotionAndPowerPulseIsBounded() {
        try (Fixture f = new Fixture()) {
            f.mode(1);
            f.op.gamepad1.a = true;
            f.op.start();
            f.op.loop();
            verify(f.drive, never()).driveRobotRelative(0.2, 0, 0);
            f.op.gamepad1.a = false;
            f.op.loop();
            f.op.gamepad1.a = true;
            f.op.loop();
            verify(f.drive).driveRobotRelative(0.2, 0, 0);
            f.time(2);
            f.op.loop();
            verify(f.op.telemetry).addLine("Power pulse complete (2 s)");
            f.time(3);
            f.op.loop();
            verify(f.drive, times(1)).driveRobotRelative(0.2, 0, 0);
        }
    }

    @Test public void headingUsesSharedGainsAndDisablesTranslation() {
        try (Fixture f = new Fixture()) {
            f.mode(6);
            f.run();
            verify(f.follower).holdPoint(argThat((Pose p) ->
                    p.getX() == 0 && p.getY() == 0 && Math.abs(p.getHeading() - Math.PI / 6) < 1e-9));
            assertEquals(f.profile.navigation.headingP,
                    f.follower.getConstants().getCoefficientsHeadingPIDF().P, 0);
            assertEquals(0, f.follower.getConstants().getCoefficientsHeadingPIDF().I, 0);
            assertTrue(f.follower.useHeading);
            assertFalse(f.follower.useTranslational);
            assertFalse(f.follower.useDrive);
            f.op.gamepad1.y = true;
            f.op.loop();
            verify(f.follower, atLeastOnce()).breakFollowing();
            verify(f.drive, atLeastOnce()).driveRobotRelative(0, 0, 0);
            f.op.loop();
            verify(f.follower, times(1)).update();
        }
    }

    @Test public void relativeLeftTargetAndPathRestoreControllers() {
        try (Fixture f = new Fixture()) {
            f.mode(8);
            when(f.tracking.getPose()).thenReturn(new FieldMap.Pose(5, 7, 90));
            f.run();
            verify(f.follower).holdPoint(argThat((Pose p) ->
                    Math.abs(p.getX() + 7) < 1e-9 && Math.abs(p.getY() - 7) < 1e-9));
            assertFalse(f.follower.useHeading);
            assertTrue(f.follower.useTranslational);
        }
        try (Fixture f = new Fixture()) {
            f.mode(9);
            f.run();
            assertTrue(f.follower.useHeading);
            assertTrue(f.follower.useDrive);
            assertTrue(f.follower.useTranslational);
        }
    }

    @Test public void trackingLossLimitsAndExceptionsStopDrive() {
        try (Fixture f = new Fixture()) {
            f.mode(1);
            f.run();
            when(f.tracking.hasValidMotion()).thenReturn(false);
            f.op.loop();
            verify(f.op.telemetry).addLine("Tracking unavailable: STOP and reinitialize");
            verify(f.drive, atLeastOnce()).driveRobotRelative(0, 0, 0);
        }
        try (Fixture f = new Fixture()) {
            f.mode(1);
            f.run();
            when(f.tracking.getPose()).thenReturn(new FieldMap.Pose(48, 0, 0));
            f.op.loop();
            verify(f.op.telemetry).addLine("Safety limit: 10 s / 48 in / 120 degrees; trial stopped");
        }
        try (Fixture f = new Fixture()) {
            f.mode(6);
            doThrow(new IllegalStateException("test")).when(f.follower).update();
            assertThrows(IllegalStateException.class, f::run);
            verify(f.drive, atLeastOnce()).driveRobotRelative(0, 0, 0);
        }
    }

    @Test public void measurementReportsPlateauAndSignedBrakeDecelerationWithoutApplyingIt() {
        try (Fixture f = new Fixture()) {
            f.mode(4);
            for (int i = 0; i < 16; i++) {
                f.op.gamepad1.dpad_right = true;
                f.op.init_loop();
                f.op.gamepad1.dpad_right = false;
                f.op.init_loop();
            }
            f.run();
            for (int i = 10; i <= 15; i++) {
                when(f.tracking.getVelocityX()).thenReturn(20.0);
                f.time(i / 10.0);
                f.op.loop();
            }
            when(f.tracking.getVelocityX()).thenReturn(10.0);
            f.time(1.6);
            f.op.loop();
            when(f.tracking.getVelocityX()).thenReturn(0.0);
            f.time(1.7);
            f.op.loop();
            verify(f.op.telemetry).addData(eq("Forward measurement"),
                    eq("20.00 in/s at 100%; acceleration -100.00 in/s^2 (full-power speed)"));
            assertEquals(f.profile.navigation.forwardZeroPowerAcceleration,
                    f.follower.getConstants().getForwardZeroPowerAcceleration(), 0);
        }
    }

    @Test public void editedGainsAreStoredInConstantsBeforeControllerRefresh() {
        try (Fixture f = new Fixture()) {
            for (int i = 0; i < 6; i++) {
                f.op.gamepad1.right_bumper = true;
                f.op.init_loop();
                f.op.gamepad1.right_bumper = false;
                f.op.init_loop();
            }
            f.op.gamepad1.dpad_right = true;
            f.op.init_loop();
            f.op.gamepad1.dpad_right = false;
            f.mode(6);
            f.run();
            assertEquals(f.profile.navigation.headingP + 0.05,
                    f.follower.getConstants().getCoefficientsHeadingPIDF().P, 1e-9);
            verify(f.follower).updateConstants();
            try {
                verify(f.stores.constructed().get(0)).save("headingP",
                        f.profile.navigation.headingP + 0.05, false);
            } catch (java.io.IOException exception) {
                throw new AssertionError(exception);
            }
        }
    }

    @Test public void initListsEveryParameterAndSaveFailureIsVisible() throws Exception {
        try (Fixture f = new Fixture()) {
            verify(f.op.telemetry).addData(eq("> testPower"), eq("%.4f"), eq(0.2));
            verify(f.op.telemetry).addData(eq("measurementPoweredSeconds"), eq("%.4f"), eq(1.5));
            verify(f.op.telemetry).addData(eq("headingP"), eq("%.4f"), eq(f.profile.navigation.headingP));
            doThrow(new java.io.IOException("disk full")).when(f.stores.constructed().get(0))
                    .save(anyString(), anyDouble(), anyBoolean());
            f.op.gamepad1.dpad_right = true;
            f.op.init_loop();
            verify(f.op.telemetry).addLine("NOT SAVED: testPower: disk full");
            verify(f.drive, never()).driveRobotRelative(0.25, 0, 0);
        }
    }

    @Test public void frozenMeasurementAndNonstationaryStartAreRejected() {
        try (Fixture f = new Fixture()) {
            f.mode(4);
            f.run();
            f.time(1.5);
            f.op.loop();
            verify(f.op.telemetry).addData(eq("Forward measurement"),
                    eq("Rejected: no stable positive speed plateau; repeat with more clearance/power"));
        }
        try (Fixture f = new Fixture()) {
            f.mode(1);
            when(f.tracking.isStationary(anyDouble(), anyDouble())).thenReturn(false);
            f.run();
            verify(f.op.telemetry).addLine("Trial refused: robot must be stationary");
            verify(f.drive, never()).driveRobotRelative(0.2, 0, 0);
        }
    }
}
