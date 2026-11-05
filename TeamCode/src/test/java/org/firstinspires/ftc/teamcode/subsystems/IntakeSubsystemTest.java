package org.firstinspires.ftc.teamcode.subsystems;

import static org.mockito.Mockito.*;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.telemetry.RobotTelemetry;
import org.junit.Test;

public class IntakeSubsystemTest {
    @Test
    public void motorOnlyModeNeverLooksUpOrCommandsServos() {
        HardwareMap hardware = mock(HardwareMap.class);
        DcMotor motor = mock(DcMotor.class);
        when(hardware.get(DcMotor.class, "intake")).thenReturn(motor);
        IntakeSubsystem intake = new IntakeSubsystem(false);

        intake.init(hardware);
        intake.setPower(0.5);
        intake.setPower(-1.5);

        verify(hardware).get(DcMotor.class, "intake");
        verifyNoMoreInteractions(hardware);
        verify(motor).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        verify(motor).setPower(0);
        verify(motor).setPower(0.5);
        verify(motor).setPower(-1);
    }

    @Test
    public void enabledServosUseCorrectedDirectionsAndClampedPower() {
        HardwareMap hardware = mock(HardwareMap.class);
        DcMotor motor = mock(DcMotor.class);
        CRServo left = mock(CRServo.class);
        CRServo right = mock(CRServo.class);
        when(hardware.get(DcMotor.class, "intake")).thenReturn(motor);
        when(hardware.get(CRServo.class, "left_intake_servo")).thenReturn(left);
        when(hardware.get(CRServo.class, "right_intake_servo")).thenReturn(right);
        IntakeSubsystem intake = new IntakeSubsystem(true);

        intake.init(hardware);
        intake.setPower(1.5);

        verify(left).setDirection(DcMotorSimple.Direction.FORWARD);
        verify(right).setDirection(DcMotorSimple.Direction.REVERSE);
        verify(left).setPower(0);
        verify(right).setPower(0);
        verify(motor).setPower(1);
        verify(left).setPower(1);
        verify(right).setPower(1);
    }

    @Test
    public void telemetryReadsActualMotorPowerWithoutCommandsOrServoLookups() {
        HardwareMap hardware = mock(HardwareMap.class);
        DcMotor motor = mock(DcMotor.class);
        when(hardware.get(DcMotor.class, "intake")).thenReturn(motor);
        IntakeSubsystem intake = new IntakeSubsystem(false);
        intake.init(hardware);
        when(motor.getPower()).thenReturn(-0.5);
        clearInvocations(hardware, motor);
        RobotTelemetry output = mock(RobotTelemetry.class);

        intake.publishTelemetry(output);

        verify(output).number("Intake", -0.5, "power");
        verify(output).value("Intake servos", "Disabled");
        verify(motor).getPower();
        verifyNoMoreInteractions(motor);
        verifyNoInteractions(hardware);
    }
}
