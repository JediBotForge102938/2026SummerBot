package org.firstinspires.ftc.teamcode.control;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.telemetry.RobotTelemetry;
import org.junit.Before;
import org.junit.Test;

public class MechanismCoordinatorTest {
    private DcMotor intakeMotor;
    private DcMotorEx flywheel;
    private CRServo windmill;
    private CRServo leftServo;
    private CRServo rightServo;
    private MechanismCoordinator mechanisms;
    private LauncherSubsystem launcher;
    private long now;

    @Before
    public void setUp() {
        HardwareMap hardware = mock(HardwareMap.class);
        intakeMotor = mock(DcMotor.class);
        flywheel = mock(DcMotorEx.class);
        windmill = mock(CRServo.class);
        leftServo = mock(CRServo.class);
        rightServo = mock(CRServo.class);
        when(hardware.get(DcMotor.class, "intake")).thenReturn(intakeMotor);
        when(hardware.get(DcMotorEx.class, "launcher")).thenReturn(flywheel);
        when(hardware.get(CRServo.class, "windmillServo")).thenReturn(windmill);
        when(hardware.get(CRServo.class, "left_intake_servo")).thenReturn(leftServo);
        when(hardware.get(CRServo.class, "right_intake_servo")).thenReturn(rightServo);
        IntakeSubsystem intake = new IntakeSubsystem();
        intake.init(hardware);
        launcher = new LauncherSubsystem(() -> now);
        launcher.init(hardware);
        mechanisms = new MechanismCoordinator(intake, launcher);
        clearInvocations(intakeMotor, flywheel, windmill, leftServo, rightServo);
    }

    @Test
    public void spinsWithoutFeedingUntilStrictlyAboveMinimumVelocity() {
        for (double velocity : new double[] {0, 1199, 1200}) {
            when(flywheel.getVelocity()).thenReturn(velocity);
            mechanisms.update(0, true);
        }
        verify(flywheel, times(3)).setVelocity(1250);
        verify(windmill, times(3)).setPower(0);
        verify(intakeMotor, times(3)).setPower(0);
    }

    @Test
    public void readyShootingFeedsWithoutTriggerInput() {
        when(flywheel.getVelocity()).thenReturn(1201.0);
        primeReadiness();
        mechanisms.update(0, true);

        verify(flywheel).setVelocity(1250);
        verify(windmill).setPower(1);
        verifyIntakePower(0.5);
    }

    @Test
    public void readyShootingClampsForwardAndPreservesReverseInteraction() {
        when(flywheel.getVelocity()).thenReturn(1250.0);
        primeReadiness();
        mechanisms.update(1, true);
        verifyIntakePower(1);

        mechanisms.update(-1, true);
        verifyIntakePower(-0.5);
        verify(windmill, times(2)).setPower(1);
    }

    @Test
    public void releaseStopsLauncherAndFeedButRetainsManualIntake() {
        when(flywheel.getVelocity()).thenReturn(1250.0);
        mechanisms.update(-1, false);

        verify(flywheel).setVelocity(0);
        verify(windmill).setPower(0);
        verifyIntakePower(-1);
    }

    @Test
    public void fallingBelowReadinessRemovesAutomaticFeed() {
        when(flywheel.getVelocity()).thenReturn(1250.0);
        primeReadiness();
        mechanisms.update(0, true);
        when(flywheel.getVelocity()).thenReturn(1175.0);
        mechanisms.update(0, true);

        verify(windmill).setPower(1);
        verify(windmill).setPower(0);
        verifyIntakePower(0.5);
        verifyIntakePower(0);
    }

    @Test
    public void stopClearsEveryMechanismEvenWhileFlywheelCoasts() {
        when(flywheel.getVelocity()).thenReturn(1250.0);
        mechanisms.stop();

        verify(flywheel).setVelocity(0);
        verify(windmill).setPower(0);
        verifyIntakePower(0);
    }

    @Test
    public void changedTargetAlsoChangesReadinessThreshold() {
        mechanisms.setLauncherTargetVelocity(1500);
        clearInvocations(windmill);
        assertEquals(1500, mechanisms.getLauncherTargetVelocity());
        assertEquals(1450, launcher.getFeedVelocityThreshold());
        when(flywheel.getVelocity()).thenReturn(1450.0);

        mechanisms.update(0, true);
        verify(windmill).setPower(0);
        verifyIntakePower(0);
        when(flywheel.getVelocity()).thenReturn(1451.0);
        primeReadiness();
        mechanisms.update(0, true);
        verify(windmill).setPower(1);
        verifyIntakePower(0.5);
        verify(flywheel).setVelocity(1500);
    }

    @Test
    public void loweringTargetUsesNewThresholdAndVelocity() {
        mechanisms.setLauncherTargetVelocity(1000);
        assertEquals(950, launcher.getFeedVelocityThreshold());
        when(flywheel.getVelocity()).thenReturn(951.0);
        primeReadiness();

        mechanisms.update(0, true);

        verify(flywheel).setVelocity(1000);
        verify(windmill).setPower(1);
        verifyIntakePower(0.5);
    }

    @Test
    public void zeroDisablesShootingAndAssistanceEvenWhileCoasting() {
        mechanisms.setLauncherTargetVelocity(0);
        clearInvocations(windmill);
        when(flywheel.getVelocity()).thenReturn(1250.0);

        mechanisms.update(0.25, true);

        verify(flywheel).setVelocity(0);
        verify(windmill).setPower(0);
        verifyIntakePower(0.25);
        assertEquals(0, launcher.getFeedVelocityThreshold());
    }

    @Test
    public void smallestPositiveTargetDoesNotFeedAtRest() {
        mechanisms.setLauncherTargetVelocity(25);
        clearInvocations(windmill);
        when(flywheel.getVelocity()).thenReturn(0.0);
        assertEquals(0, launcher.getFeedVelocityThreshold());

        mechanisms.update(0, true);
        verify(windmill).setPower(0);
        verifyIntakePower(0);
        when(flywheel.getVelocity()).thenReturn(25.0);
        primeReadiness();
        mechanisms.update(0, true);
        verify(windmill).setPower(1);
        verifyIntakePower(0.5);
    }

    @Test
    public void unreachableTargetDoesNotFeed() {
        mechanisms.setLauncherTargetVelocity(6000);
        clearInvocations(windmill);
        when(flywheel.getVelocity()).thenReturn(3000.0);

        mechanisms.update(0, true);

        verify(flywheel).setVelocity(6000);
        verify(windmill).setPower(0);
        verifyIntakePower(0);
    }

    @Test
    public void invalidTargetIsRejectedWithoutChangingCurrentTarget() {
        assertThrows(IllegalArgumentException.class, () -> launcher.setTargetVelocity(-1));
        assertThrows(IllegalArgumentException.class, () -> launcher.setTargetVelocity(6001));
        assertEquals(1250, launcher.getTargetVelocity());
    }

    private void verifyIntakePower(double power) {
        verify(intakeMotor).setPower(power);
        verify(leftServo).setPower(power);
        verify(rightServo).setPower(power);
    }

    private void primeReadiness() {
        mechanisms.update(0, true);
        now += 200_000_000L;
        clearInvocations(intakeMotor, flywheel, windmill, leftServo, rightServo);
    }

    @Test
    public void requiresFullContinuousDwellAndResetsTimerOnSpeedDip() {
        when(flywheel.getVelocity()).thenReturn(1250.0);
        mechanisms.update(0, true);
        assertEquals(LauncherSubsystem.Status.SPINNING_UP, launcher.getStatus());
        now = 199_000_000L;
        mechanisms.update(0, true);
        assertFalse(launcher.isReady());
        when(flywheel.getVelocity()).thenReturn(1200.0);
        mechanisms.update(0, true);
        when(flywheel.getVelocity()).thenReturn(1250.0);
        now = 200_000_000L;
        mechanisms.update(0, true);
        now = 399_000_000L;
        mechanisms.update(0, true);
        assertFalse(launcher.isReady());
        now = 400_000_000L;
        mechanisms.update(0, true);
        assertTrue(launcher.isReady());
        assertEquals(LauncherSubsystem.Status.READY, launcher.getStatus());
        verify(windmill).setPower(1);
        verifyIntakePower(0.5);
    }

    @Test
    public void hysteresisHoldsReadinessUntilDropoutThenRequiresFullRequalification() {
        when(flywheel.getVelocity()).thenReturn(1250.0);
        primeReadiness();
        mechanisms.update(0, true);
        when(flywheel.getVelocity()).thenReturn(1176.0);
        mechanisms.update(0, true);
        assertTrue(launcher.isReady());
        when(flywheel.getVelocity()).thenReturn(1175.0);
        mechanisms.update(0, true);
        assertFalse(launcher.isReady());
        assertEquals(LauncherSubsystem.Status.BELOW_TARGET, launcher.getStatus());

        when(flywheel.getVelocity()).thenReturn(1201.0);
        now += 1_000_000_000L;
        mechanisms.update(0, true);
        assertFalse(launcher.isReady());
        now += 200_000_000L;
        mechanisms.update(0, true);
        assertTrue(launcher.isReady());
    }

    @Test
    public void releaseAndTargetChangeImmediatelyDisarmAndRequireNewDwell() {
        when(flywheel.getVelocity()).thenReturn(1250.0);
        primeReadiness();
        mechanisms.update(0, true);
        assertTrue(launcher.isReady());
        mechanisms.update(0, false);
        assertEquals(LauncherSubsystem.Status.OFF, launcher.getStatus());
        assertFalse(launcher.isReady());
        now += 1_000_000_000L;
        mechanisms.update(0, true);
        assertFalse(launcher.isReady());
        now += 200_000_000L;
        mechanisms.update(0, true);
        assertTrue(launcher.isReady());

        clearInvocations(windmill);
        mechanisms.setLauncherTargetVelocity(1275);
        assertFalse(launcher.isReady());
        verify(windmill).setPower(0);
        now += 1_000_000_000L;
        mechanisms.update(0, true);
        assertFalse(launcher.isReady());
        now += 200_000_000L;
        mechanisms.update(0, true);
        assertTrue(launcher.isReady());
    }

    @Test
    public void stoppingWhileReadyClearsOutputsAndReadiness() {
        when(flywheel.getVelocity()).thenReturn(1250.0);
        primeReadiness();
        mechanisms.update(0, true);
        clearInvocations(flywheel, windmill, intakeMotor, leftServo, rightServo);

        mechanisms.stop();

        assertFalse(mechanisms.isLauncherReady());
        assertEquals(LauncherSubsystem.Status.OFF, launcher.getStatus());
        verify(flywheel).setVelocity(0);
        verify(windmill).setPower(0);
        verifyIntakePower(0);
    }

    @Test
    public void launcherTelemetryIsReadOnlyAndPublishesReadinessSeverity() {
        RobotTelemetry output = mock(RobotTelemetry.class);
        launcher.publishTelemetry(output);
        verify(output).status("Launcher", "OFF", RobotTelemetry.Level.INFO);
        verify(output).number("Launcher target", 1250, "ticks/s");
        verify(flywheel, never()).setVelocity(anyDouble());
        verify(windmill, never()).setPower(anyDouble());

        when(flywheel.getVelocity()).thenReturn(1250.0);
        mechanisms.update(0, true);
        launcher.publishTelemetry(output);
        verify(output).status("Launcher", "SPINNING UP", RobotTelemetry.Level.WARNING);
        now += 200_000_000L;
        mechanisms.update(0, true);
        clearInvocations(flywheel, windmill);
        launcher.publishTelemetry(output);
        verify(output).status("Launcher", "READY", RobotTelemetry.Level.SUCCESS);
        verify(flywheel, never()).setVelocity(anyDouble());
        verify(windmill, never()).setPower(anyDouble());

        when(flywheel.getVelocity()).thenReturn(1175.0);
        mechanisms.update(0, true);
        launcher.publishTelemetry(output);
        verify(output).status("Launcher", "BELOW TARGET", RobotTelemetry.Level.WARNING);
    }

    @Test
    public void zeroTargetTelemetryReportsDisabledFeed() {
        launcher.setTargetVelocity(0);
        RobotTelemetry output = mock(RobotTelemetry.class);

        launcher.publishTelemetry(output);

        verify(output).number("Launcher target", 0, "ticks/s");
        verify(output).value("Launcher feed", "Disabled");
    }
}
