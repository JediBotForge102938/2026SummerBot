package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.control.MechanismCoordinator;
import org.firstinspires.ftc.teamcode.telemetry.RobotTelemetry;
import org.junit.Before;
import org.junit.Test;

import java.util.Arrays;
import java.util.Collections;
import java.io.File;

public class LauncherSubsystemTest {
    private HardwareMap hardware;
    private DcMotorEx left;
    private DcMotorEx right;
    private LauncherSubsystem launcher;
    private long now;

    @Before
    public void setUp() {
        hardware = mock(HardwareMap.class);
        left = mock(DcMotorEx.class);
        right = mock(DcMotorEx.class);
        when(hardware.get(DcMotorEx.class, "left_launcher")).thenReturn(left);
        when(hardware.get(DcMotorEx.class, "right_launcher")).thenReturn(right);
        launcher = new LauncherSubsystem(Arrays.asList(
                new LauncherSubsystem.MotorConfig("left_launcher", DcMotor.Direction.REVERSE),
                new LauncherSubsystem.MotorConfig("right_launcher", DcMotor.Direction.FORWARD)),
                () -> now);
        launcher.init(hardware);
        clearInvocations(left, right);
    }

    @Test
    public void configuresDirectionsAndSharedVelocityWithoutRequiringFeederHardware() {
        LauncherSubsystem configured = new LauncherSubsystem(Arrays.asList(
                new LauncherSubsystem.MotorConfig("left_launcher", DcMotor.Direction.REVERSE),
                new LauncherSubsystem.MotorConfig("right_launcher", DcMotor.Direction.FORWARD)));
        configured.init(hardware);
        verify(left).setDirection(DcMotor.Direction.REVERSE);
        verify(right).setDirection(DcMotor.Direction.FORWARD);
        verify(left).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        verify(right).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setTargetVelocity(1500);
        launcher.update(true);
        verify(left).setVelocity(1500);
        verify(right).setVelocity(1500);
        verify(hardware, never()).get(eq(com.qualcomm.robotcore.hardware.CRServo.class), anyString());
    }

    @Test
    public void launcherUsesDirectionsFromEachRobotProfile() {
        RobotProfile competition = RobotProfile.load(new File("../robot-configs"),
                "biobuzz-competitionbot", "competitionbot");
        DcMotorEx competitionMotor = mock(DcMotorEx.class);
        when(hardware.get(DcMotorEx.class, "launcher")).thenReturn(competitionMotor);
        LauncherSubsystem configured =
                new LauncherSubsystem(competition.launcherMotors, competition.launcherPidf,
                        competition.launcherFeedVelocityMargin);
        configured.init(hardware);
        verify(competitionMotor).setDirection(competition.launcherMotors.get(0).direction);
        when(competitionMotor.getVelocity()).thenReturn(1250.0);
        configured.update(true);
        verify(competitionMotor).setVelocity(1250);
        assertEquals(1250, configured.getVelocity(), 0);

        DcMotorEx starterMotor = mock(DcMotorEx.class);
        when(hardware.get(DcMotorEx.class, "launcher")).thenReturn(starterMotor);
        RobotProfile starter = RobotProfile.load(new File("../robot-configs"),
                "biobuzz-starterbot", "starterbot");
        new LauncherSubsystem(starter.launcherMotors, starter.launcherPidf,
                starter.launcherFeedVelocityMargin).init(hardware);
        verify(starterMotor).setDirection(DcMotor.Direction.FORWARD);
    }

    @Test
    public void competitionMarginGatesReadinessAndDropoutAtConfiguredThresholds() {
        RobotProfile competition = RobotProfile.load(new File("../robot-configs"),
                "biobuzz-competitionbot", "competitionbot");
        DcMotorEx motor = mock(DcMotorEx.class);
        when(hardware.get(DcMotorEx.class, "launcher")).thenReturn(motor);
        LauncherSubsystem configured = new LauncherSubsystem(competition.launcherMotors,
                competition.launcherPidf, competition.launcherFeedVelocityMargin, () -> now);
        configured.init(hardware);
        configured.setTargetVelocity(800);
        int threshold = 800 - competition.launcherFeedVelocityMargin;
        int dropout = threshold - 25;
        assertEquals(threshold, configured.getFeedVelocityThreshold());
        assertEquals(dropout, configured.getFeedDropoutThreshold());

        when(motor.getVelocity()).thenReturn((double) threshold);
        assertFalse(configured.update(true));
        now += 200_000_000L;
        assertFalse(configured.update(true));
        when(motor.getVelocity()).thenReturn((double) threshold + 1);
        assertFalse(configured.update(true));
        now += 199_000_000L;
        assertFalse(configured.update(true));
        now += 1_000_000L;
        assertTrue(configured.update(true));
        when(motor.getVelocity()).thenReturn((double) dropout + 1);
        assertTrue(configured.update(true));
        when(motor.getVelocity()).thenReturn((double) dropout);
        assertFalse(configured.update(true));
        assertEquals(LauncherSubsystem.Status.BELOW_TARGET, configured.getStatus());
        configured.setTargetVelocity(0);
        assertFalse(configured.update(true));
    }

    @Test
    public void configuredPidfIsAppliedToEveryLauncherMotor() {
        LauncherSubsystem.PidfConfig pidf = new LauncherSubsystem.PidfConfig(20, 1.5, 2, 8);
        LauncherSubsystem configured = new LauncherSubsystem(Arrays.asList(
                new LauncherSubsystem.MotorConfig("left_launcher", DcMotor.Direction.REVERSE),
                new LauncherSubsystem.MotorConfig("right_launcher", DcMotor.Direction.FORWARD)), pidf);
        configured.init(hardware);
        for (DcMotorEx motor : Arrays.asList(left, right)) {
            org.mockito.ArgumentCaptor<PIDFCoefficients> coefficients =
                    org.mockito.ArgumentCaptor.forClass(PIDFCoefficients.class);
            verify(motor).setPIDFCoefficients(eq(DcMotor.RunMode.RUN_USING_ENCODER),
                    coefficients.capture());
            assertEquals(20, coefficients.getValue().p, 0);
            assertEquals(1.5, coefficients.getValue().i, 0);
            assertEquals(2, coefficients.getValue().d, 0);
            assertEquals(8, coefficients.getValue().f, 0);
        }
    }

    @Test
    public void bothMotorsMustQualifyTogetherAndEitherMotorCanResetDwell() {
        when(left.getVelocity()).thenReturn(1250.0);
        when(right.getVelocity()).thenReturn(1200.0);
        assertFalse(launcher.update(true));
        now = 1_000_000_000L;
        assertFalse(launcher.update(true));
        when(right.getVelocity()).thenReturn(1250.0);
        assertFalse(launcher.update(true));
        now += 199_000_000L;
        assertFalse(launcher.update(true));
        when(left.getVelocity()).thenReturn(1200.0);
        assertFalse(launcher.update(true));
        when(left.getVelocity()).thenReturn(1250.0);
        assertFalse(launcher.update(true));
        now += 199_000_000L;
        assertFalse(launcher.update(true));
        now += 1_000_000L;
        assertTrue(launcher.update(true));
    }

    @Test
    public void eitherMotorDropoutDisarmsAndAllMustRequalify() {
        when(left.getVelocity()).thenReturn(1250.0);
        when(right.getVelocity()).thenReturn(1250.0);
        launcher.update(true);
        now += 200_000_000L;
        assertTrue(launcher.update(true));
        for (DcMotorEx motor : Arrays.asList(left, right)) {
            when(motor.getVelocity()).thenReturn(1176.0);
            assertTrue(launcher.update(true));
            when(motor.getVelocity()).thenReturn(1175.0);
            assertFalse(launcher.update(true));
            assertEquals(LauncherSubsystem.Status.BELOW_TARGET, launcher.getStatus());
            when(motor.getVelocity()).thenReturn(1250.0);
            assertFalse(launcher.update(true));
            now += 200_000_000L;
            assertTrue(launcher.update(true));
        }
        launcher.update(false);
        verify(left).setVelocity(0);
        verify(right).setVelocity(0);
        assertFalse(launcher.isReady());
        assertFalse(launcher.update(true));
        launcher.setTargetVelocity(0);
        assertFalse(launcher.update(true));
        verify(left, times(2)).setVelocity(0);
        verify(right, times(2)).setVelocity(0);
    }

    @Test
    public void invalidConfigurationAndMissingHardwareFailRatherThanUsingPartialLauncher() {
        assertThrows(IllegalArgumentException.class,
                () -> new LauncherSubsystem(Collections.emptyList()));
        assertThrows(IllegalArgumentException.class,
                () -> new LauncherSubsystem.MotorConfig(" ", DcMotor.Direction.FORWARD));
        LauncherSubsystem.MotorConfig motor =
                new LauncherSubsystem.MotorConfig("duplicate", DcMotor.Direction.FORWARD);
        assertThrows(IllegalArgumentException.class,
                () -> new LauncherSubsystem(Arrays.asList(motor, motor)));
        when(hardware.get(DcMotorEx.class, "right_launcher")).thenThrow(
                new IllegalArgumentException("Missing right_launcher"));
        LauncherSubsystem missing = new LauncherSubsystem(Arrays.asList(
                new LauncherSubsystem.MotorConfig("left_launcher", DcMotor.Direction.REVERSE),
                new LauncherSubsystem.MotorConfig("right_launcher", DcMotor.Direction.FORWARD)));
        assertThrows(IllegalArgumentException.class, () -> missing.init(hardware));
        verify(left, never()).setVelocity(anyDouble());
    }

    @Test
    public void invalidVelocityStopsBothMotorsAndReportsError() {
        when(left.getVelocity()).thenReturn(1250.0);
        when(right.getVelocity()).thenReturn(1250.0);
        launcher.update(true);
        now += 200_000_000L;
        assertTrue(launcher.update(true));
        when(right.getVelocity()).thenReturn(Double.NaN);
        assertThrows(IllegalStateException.class, () -> launcher.update(true));
        assertFalse(launcher.isReady());
        verify(left).setVelocity(0);
        verify(right).setVelocity(0);
    }

    @Test
    public void coordinatorFeedsOnlyWhenBothReadyAndStopsOnTargetChangeOrInvalidSpeed() {
        IntakeSubsystem intake = mock(IntakeSubsystem.class);
        WindmillSubsystem feeder = mock(WindmillSubsystem.class);
        MechanismCoordinator coordinator = new MechanismCoordinator(intake, launcher, feeder);
        when(left.getVelocity()).thenReturn(1500.0);
        when(right.getVelocity()).thenReturn(1200.0);
        coordinator.update(0, true);
        now += 200_000_000L;
        coordinator.update(0, true);
        verify(feeder, never()).update(true);

        when(right.getVelocity()).thenReturn(1500.0);
        coordinator.update(0, true);
        now += 200_000_000L;
        coordinator.update(0, true);
        verify(feeder).update(true);
        verify(intake).setPower(0.5);
        clearInvocations(feeder);
        coordinator.setLauncherTargetVelocity(1400);
        assertFalse(launcher.isReady());
        verify(feeder).update(false);
        coordinator.update(0, true);
        assertFalse(launcher.isReady());
        now += 200_000_000L;
        coordinator.update(0, true);
        assertTrue(launcher.isReady());

        clearInvocations(feeder, intake);
        when(left.getVelocity()).thenReturn(Double.NaN);
        assertThrows(IllegalStateException.class, () -> coordinator.update(0, true));
        verify(feeder).update(false);
        verify(intake).setPower(0);
    }

    @Test
    public void telemetryShowsEachMotorAndSlowestSpeedWithoutChangingOutputs() {
        when(left.getVelocity()).thenReturn(1250.0);
        when(right.getVelocity()).thenReturn(1150.0);
        RobotTelemetry output = mock(RobotTelemetry.class);
        launcher.publishTelemetry(output);
        verify(output).number("Launcher left_launcher", 1250, "ticks/s");
        verify(output).number("Launcher right_launcher", 1150, "ticks/s");
        verify(output).number("Launcher actual", 1150, "ticks/s");
        verify(output).status("Launcher", "OFF", RobotTelemetry.Level.INFO);
        verify(left, never()).setVelocity(anyDouble());
        verify(right, never()).setVelocity(anyDouble());
    }

    @Test
    public void disabledFeederAllowsSharedCoordinatorWithoutLookingUpStarterServo() {
        IntakeSubsystem intake = mock(IntakeSubsystem.class);
        MechanismCoordinator coordinator =
                new MechanismCoordinator(intake, launcher, LauncherFeeder.DISABLED);
        when(left.getVelocity()).thenReturn(1250.0);
        when(right.getVelocity()).thenReturn(1250.0);
        coordinator.update(0, true);
        now += 200_000_000L;
        coordinator.update(0, true);
        assertTrue(coordinator.isLauncherReady());
        verify(intake).setPower(0.5);
        coordinator.stop();
        verify(left).setVelocity(0);
        verify(right).setVelocity(0);
        assertFalse(coordinator.isLauncherReady());
    }
}
