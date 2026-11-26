package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.RobotProfile;
import org.junit.Test;
import org.mockito.ArgumentCaptor;
import org.mockito.MockedStatic;

import java.io.File;

public class LauncherTuningTeleOpTest {
    private final DcMotorEx motor = mock(DcMotorEx.class);
    private final Servo gate = mock(Servo.class);

    private LauncherTuningTeleOp initialize(RobotProfile profile) {
        LauncherTuningTeleOp op = new LauncherTuningTeleOp();
        op.hardwareMap = mock(HardwareMap.class);
        op.telemetry = mock(Telemetry.class);
        op.gamepad1 = mock(Gamepad.class);
        when(op.hardwareMap.get(DcMotorEx.class, "launcher")).thenReturn(motor);
        when(op.hardwareMap.get(Servo.class, "gate")).thenReturn(gate);
        try (MockedStatic<RobotProfile> profiles = mockStatic(RobotProfile.class)) {
            profiles.when(() -> RobotProfile.loadActive(null)).thenReturn(profile);
            op.init();
            profiles.verify(() -> RobotProfile.loadActive(null));
        }
        return op;
    }

    private RobotProfile profile(String robot) {
        return RobotProfile.load(new File("../robot-configs"), "biobuzz-" + robot, robot);
    }

    private PIDFCoefficients latestPidf() {
        ArgumentCaptor<PIDFCoefficients> coefficients =
                ArgumentCaptor.forClass(PIDFCoefficients.class);
        verify(motor, atLeastOnce()).setPIDFCoefficients(eq(DcMotor.RunMode.RUN_USING_ENCODER),
                coefficients.capture());
        return coefficients.getValue();
    }

    private void nextRow(LauncherTuningTeleOp op) {
        op.gamepad1.dpad_down = true;
        op.loop();
        op.gamepad1.dpad_down = false;
        op.loop();
    }

    private void increase(LauncherTuningTeleOp op) {
        op.gamepad1.dpad_right = true;
        op.loop();
        op.gamepad1.dpad_right = false;
        op.loop();
    }

    @Test
    public void detectsEitherProfileAndAppliesPidfAndDirectionWithoutSpinningOrFeeding() {
        for (String robot : new String[] {"starterbot", "competitionbot"}) {
            reset(motor, gate);
            RobotProfile profile = profile(robot);
            LauncherTuningTeleOp op = initialize(profile);
            PIDFCoefficients coefficients = latestPidf();
            assertEquals(profile.launcherPidf.p, coefficients.p, 0);
            assertEquals(profile.launcherPidf.i, coefficients.i, 0);
            assertEquals(profile.launcherPidf.d, coefficients.d, 0);
            assertEquals(profile.launcherPidf.f, coefficients.f, 0);
            verify(motor).setDirection(profile.launcherMotors.get(0).direction);
            verify(motor).setVelocity(0);
            verify(op.telemetry).addData("Robot profile", profile.name);
            op.init_loop();
            op.start();
            op.loop();
            op.stop();
            verify(motor, never()).setVelocity(1200);
            if (profile.launcherGate != null) {
                verify(gate, times(2)).setPosition(profile.launcherGate.closedPosition);
                verify(gate, never()).setPosition(profile.launcherGate.openPosition);
            }
            verify(op.hardwareMap, never()).get(eq(DcMotorEx.class), eq("intake"));
        }
    }

    @Test
    public void startRequiresBumperReleaseAndReleaseOrStopStopsMotorWithActualTargetTelemetry() {
        LauncherTuningTeleOp op = initialize(profile("competitionbot"));
        op.gamepad1.right_bumper = true;
        op.start();
        op.loop();
        verify(motor, never()).setVelocity(1200);
        op.gamepad1.right_bumper = false;
        op.loop();
        clearInvocations(motor, op.telemetry);
        when(motor.getVelocity()).thenReturn(1140.0);
        op.gamepad1.right_bumper = true;
        op.loop();
        verify(motor).setVelocity(1200);
        verify(op.telemetry).addData("Actual / target", "%.1f / %d ticks/s", 1140.0, 1200);
        verify(op.telemetry).addData("Error (target - actual)", "%.1f ticks/s", 60.0);
        verify(op.telemetry).addData("Tracking (+/-50 ticks/s)", "OUTSIDE");
        op.gamepad1.right_bumper = false;
        op.loop();
        verify(motor).setVelocity(0);
        clearInvocations(motor);
        op.gamepad1.right_bumper = true;
        op.loop();
        op.stop();
        verify(motor).setVelocity(0);
    }

    @Test
    public void trackingUsesDesiredFiftyTickToleranceRatherThanProfileReadinessMargin() {
        LauncherTuningTeleOp op = initialize(profile("competitionbot"));
        op.start();
        op.gamepad1.right_bumper = true;
        for (double actual : new double[] {1150, 1250, 1149, 1251}) {
            when(motor.getVelocity()).thenReturn(actual);
            clearInvocations(op.telemetry);
            op.loop();
            verify(op.telemetry).addData("Tracking (+/-50 ticks/s)",
                    Math.abs(1200 - actual) <= 50 ? "WITHIN" : "OUTSIDE");
        }
        op.gamepad1.right_bumper = false;
        when(motor.getVelocity()).thenReturn(1200.0);
        clearInvocations(op.telemetry);
        op.loop();
        verify(op.telemetry).addData("Tracking (+/-50 ticks/s)", "OFF");
        op.stop();
    }

    @Test
    public void adjustsEachPidfRowLiveWithFineStepsAndNonnegativeLimits() {
        RobotProfile profile = profile("starterbot");
        LauncherTuningTeleOp op = initialize(profile);
        op.start();
        op.gamepad1.right_bumper = true;
        nextRow(op);
        increase(op);
        nextRow(op);
        increase(op);
        nextRow(op);
        increase(op);
        nextRow(op);
        op.gamepad1.left_bumper = true;
        increase(op);
        PIDFCoefficients coefficients = latestPidf();
        assertEquals(profile.launcherPidf.p + 1, coefficients.p, 1e-9);
        assertEquals(profile.launcherPidf.i + 0.01, coefficients.i, 1e-9);
        assertEquals(profile.launcherPidf.d + 0.1, coefficients.d, 1e-9);
        assertEquals(profile.launcherPidf.f + 0.01, coefficients.f, 1e-9);
        verify(motor, atLeastOnce()).setVelocity(1200);
        nextRow(op);
        nextRow(op);
        nextRow(op);
        op.gamepad1.left_bumper = false;
        for (int n = 0; n < 2; n++) {
            op.gamepad1.dpad_left = true;
            op.loop();
            op.gamepad1.dpad_left = false;
            op.loop();
        }
        assertEquals(0, latestPidf().i, 0);
        op.stop();
    }

    @Test
    public void arrowsRequireFreshPressAndOpposingArrowsCancelAndTargetIsBounded() {
        LauncherTuningTeleOp op = initialize(profile("starterbot"));
        op.gamepad1.dpad_right = true;
        op.init_loop();
        op.start();
        op.gamepad1.right_bumper = true;
        op.loop();
        verify(motor).setVelocity(1200);
        op.gamepad1.dpad_right = false;
        op.loop();
        clearInvocations(motor);
        op.gamepad1.dpad_right = true;
        op.loop();
        op.loop();
        verify(motor, times(2)).setVelocity(1225);
        op.gamepad1.dpad_right = false;
        op.loop();
        op.gamepad1.dpad_left = op.gamepad1.dpad_right = true;
        op.loop();
        op.gamepad1.dpad_left = op.gamepad1.dpad_right = false;
        op.loop();
        clearInvocations(motor);
        for (int n = 0; n < 50; n++) {
            op.gamepad1.dpad_left = true;
            op.loop();
            op.gamepad1.dpad_left = false;
            op.loop();
        }
        verify(motor, atLeastOnce()).setVelocity(0);
        clearInvocations(motor);
        for (int n = 0; n < 241; n++) {
            increase(op);
        }
        verify(motor, atLeastOnce()).setVelocity(6000);
        verify(motor, never()).setVelocity(doubleThat(value -> value < 0 || value > 6000));
        op.stop();
    }
}
