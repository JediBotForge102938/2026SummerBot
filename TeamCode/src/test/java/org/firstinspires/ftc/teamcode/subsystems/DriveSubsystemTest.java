package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.telemetry.RobotTelemetry;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.mockito.MockedConstruction;

public class DriveSubsystemTest {
    private HardwareMap hardware;
    private IMU imu;
    private DriveSubsystem drive;
    private MockedConstruction<RevHubOrientationOnRobot> orientation;

    @Before
    public void setUp() {
        // The SDK orientation constructor uses android.opengl.Matrix, absent on the test JVM.
        orientation = mockConstruction(RevHubOrientationOnRobot.class);
        hardware = mock(HardwareMap.class);
        imu = mock(IMU.class);
        when(hardware.get(eq(DcMotor.class), anyString())).thenAnswer(
                invocation -> mock(DcMotor.class));
        when(hardware.get(IMU.class, "imu")).thenReturn(imu);
        when(imu.initialize(any(IMU.Parameters.class))).thenReturn(true);
        drive = new DriveSubsystem();
    }

    @After
    public void tearDown() {
        orientation.close();
    }

    @Test
    public void initializationFailureIsNotSilentlyIgnored() {
        when(imu.initialize(any(IMU.Parameters.class))).thenReturn(false);
        assertThrows(IllegalStateException.class, () -> drive.init(hardware));
    }

    @Test
    public void missingImuIsNotSilentlyIgnoredForRobotCentricDriving() {
        when(hardware.get(IMU.class, "imu")).thenThrow(
                new IllegalArgumentException("No configured imu"));
        assertThrows(IllegalArgumentException.class, () -> drive.init(hardware));
    }

    @Test
    public void robotRelativeMixingNormalizesWithCorrectedStrafe() {
        drive.init(hardware);
        drive.driveRobotRelative(1, 1, 1);

        assertEquals(1.0 / 3, drive.getLeftFrontPower(), 1e-9);
        assertEquals(1.0 / 3, drive.getRightFrontPower(), 1e-9);
        assertEquals(1, drive.getLeftBackPower(), 1e-9);
        assertEquals(-1.0 / 3, drive.getRightBackPower(), 1e-9);
    }

    @Test
    public void fieldRelativeUsesHeadingAndExplicitResetUsesImu() {
        drive.init(hardware);
        when(imu.getRobotYawPitchRollAngles()).thenReturn(
                new YawPitchRollAngles(AngleUnit.DEGREES, 90, 0, 0, 0));
        drive.driveFieldRelative(1, 0, 0);

        assertEquals(-1, drive.getLeftFrontPower(), 1e-9);
        assertEquals(1, drive.getRightFrontPower(), 1e-9);
        assertEquals(1, drive.getLeftBackPower(), 1e-9);
        assertEquals(-1, drive.getRightBackPower(), 1e-9);
        verify(imu, never()).resetYaw();

        drive.resetHeading();
        verify(imu).resetYaw();
    }

    @Test
    public void newDriveInstanceKeepsExistingImuHeadingWithoutResettingYaw() {
        when(imu.getRobotYawPitchRollAngles()).thenReturn(
                new YawPitchRollAngles(AngleUnit.DEGREES, 90, 0, 0, 0));
        drive.init(hardware);
        assertEquals(90, drive.getHeadingDegrees(), 1e-9);

        DriveSubsystem restartedDrive = new DriveSubsystem();
        restartedDrive.init(hardware);
        assertEquals(90, restartedDrive.getHeadingDegrees(), 1e-9);
        restartedDrive.driveFieldRelative(1, 0, 0);
        assertEquals(-1, restartedDrive.getLeftFrontPower(), 1e-9);
        assertEquals(1, restartedDrive.getRightFrontPower(), 1e-9);
        assertEquals(1, restartedDrive.getLeftBackPower(), 1e-9);
        assertEquals(-1, restartedDrive.getRightBackPower(), 1e-9);
        verify(imu, never()).resetYaw();
    }

    @Test
    public void precisionScalesAfterNormalizationRatherThanGettingNormalizedAway() {
        drive.init(hardware);
        drive.driveRobotRelative(1, 1, 1, 0.35);
        assertEquals(0.35 / 3, drive.getLeftFrontPower(), 1e-9);
        assertEquals(0.35 / 3, drive.getRightFrontPower(), 1e-9);
        assertEquals(0.35, drive.getLeftBackPower(), 1e-9);
        assertEquals(-0.35 / 3, drive.getRightBackPower(), 1e-9);

        drive.driveRobotRelative(1, 1, 1);
        assertEquals(1, drive.getLeftBackPower(), 1e-9);
    }

    @Test
    public void fieldRelativePrecisionScalesTransformedAndNormalizedWheelPowers() {
        drive.init(hardware);
        when(imu.getRobotYawPitchRollAngles()).thenReturn(
                new YawPitchRollAngles(AngleUnit.DEGREES, 90, 0, 0, 0));
        drive.driveFieldRelative(1, 0, 1, 0.35);
        assertEquals(0, drive.getLeftFrontPower(), 1e-9);
        assertEquals(0, drive.getRightFrontPower(), 1e-9);
        assertEquals(0.35, drive.getLeftBackPower(), 1e-9);
        assertEquals(-0.35, drive.getRightBackPower(), 1e-9);
    }

    @Test
    public void telemetryPublishesHeadingAndAppliedMotorPowerWithoutCommands() {
        drive.init(hardware);
        when(imu.getRobotYawPitchRollAngles()).thenReturn(
                new YawPitchRollAngles(AngleUnit.DEGREES, 45, 0, 0, 0));
        drive.driveRobotRelative(1, 0, 0, 0.35);
        clearInvocations(imu);
        RobotTelemetry output = mock(RobotTelemetry.class);

        drive.publishTelemetry(output);

        verify(output).number("Heading", 45, "degrees");
        verify(output).number("Drive front left", 0.35, "power");
        verify(output).number("Drive front right", 0.35, "power");
        verify(output).number("Drive back left", 0.35, "power");
        verify(output).number("Drive back right", 0.35, "power");
        verify(imu, never()).resetYaw();
        assertEquals(0.35, drive.getLeftFrontPower(), 1e-9);
    }

    @Test
    public void onlyLateralPowerPatternIsReversed() {
        drive.init(hardware);
        drive.driveRobotRelative(0, 1, 0);
        assertWheelPowers(-1, 1, 1, -1);
        drive.driveRobotRelative(0, -1, 0);
        assertWheelPowers(1, -1, -1, 1);
        drive.driveRobotRelative(1, 0, 0);
        assertWheelPowers(1, 1, 1, 1);
        drive.driveRobotRelative(-1, 0, 0);
        assertWheelPowers(-1, -1, -1, -1);
        drive.driveRobotRelative(0, 0, 1);
        assertWheelPowers(1, -1, 1, -1);
        drive.driveRobotRelative(0, 0, -1);
        assertWheelPowers(-1, 1, -1, 1);
    }

    @Test
    public void fieldRelativeStrafeIsCorrectedAtZeroHeadingToo() {
        drive.init(hardware);
        when(imu.getRobotYawPitchRollAngles()).thenReturn(
                new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0));
        drive.driveFieldRelative(0, 1, 0);
        assertWheelPowers(-1, 1, 1, -1);
        drive.driveFieldRelative(0, -1, 0);
        assertWheelPowers(1, -1, -1, 1);
    }

    @Test
    public void prioritizedStrafePreservesRotationInsteadOfNormalizingItAway() {
        drive.init(hardware);
        drive.driveRobotRelative(0, 1, 0.6, 1, true);
        assertWheelPowers(0.2, -0.2, 1, -1);
        drive.driveRobotRelative(0, 1, -0.6, 1, true);
        assertWheelPowers(-1, 1, -0.2, 0.2);
    }

    @Test
    public void rotationPriorityPreservesTurnAndTranslationDirectionAcrossInputRange() {
        drive.init(hardware);
        double[] inputs = {-1, -0.5, 0, 0.5, 1};
        for (double forward : inputs) {
            for (double strafe : inputs) {
                for (double rotate : inputs) {
                    for (double powerScale : new double[] {1, 0.35}) {
                        drive.driveRobotRelative(forward, strafe, rotate, powerScale, true);
                        double fl = drive.getLeftFrontPower();
                        double fr = drive.getRightFrontPower();
                        double bl = drive.getLeftBackPower();
                        double br = drive.getRightBackPower();
                        assertEquals(rotate * powerScale, (fl - fr + bl - br) / 4, 1e-9);
                        double translation = Math.abs(forward) + Math.abs(strafe);
                        double scale = translation == 0 ? 1
                                : Math.min(1, (1 - Math.abs(rotate)) / translation);
                        assertEquals(forward * scale * powerScale, (fl + fr + bl + br) / 4, 1e-9);
                        assertEquals(-strafe * scale * powerScale, (fl - fr - bl + br) / 4, 1e-9);
                        for (double wheel : new double[] {fl, fr, bl, br}) {
                            assertTrue(Math.abs(wheel) <= powerScale + 1e-9);
                        }
                    }
                }
            }
        }
    }

    @Test
    public void fieldRelativePriorityReservesRotationAfterHeadingTransform() {
        drive.init(hardware);
        when(imu.getRobotYawPitchRollAngles()).thenReturn(
                new YawPitchRollAngles(AngleUnit.DEGREES, 90, 0, 0, 0));
        drive.driveFieldRelative(1, 0, 0.6, 1, true);
        assertWheelPowers(0.2, -0.2, 1, -1);
        drive.driveFieldRelative(1, 0, 0.6, 0.35, true);
        assertWheelPowers(0.07, -0.07, 0.35, -0.35);
    }

    @Test
    public void priorityDoesNotReduceTranslationWhenThereIsHeadroom() {
        drive.init(hardware);
        drive.driveRobotRelative(0.1, 0.2, 0.3, 1, true);
        assertWheelPowers(0.2, 0, 0.6, -0.4);
        assertThrows(IllegalArgumentException.class,
                () -> drive.driveRobotRelative(0, 0, 1.1, 1, true));
    }

    private void assertWheelPowers(double fl, double fr, double bl, double br) {
        assertEquals(fl, drive.getLeftFrontPower(), 1e-9);
        assertEquals(fr, drive.getRightFrontPower(), 1e-9);
        assertEquals(bl, drive.getLeftBackPower(), 1e-9);
        assertEquals(br, drive.getRightBackPower(), 1e-9);
    }
}
