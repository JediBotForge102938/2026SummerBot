package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.telemetry.RobotTelemetry;
import org.firstinspires.ftc.teamcode.telemetry.TelemetrySource;

public class IntakeSubsystem implements TelemetrySource {
    private final boolean servosEnabled;
    private DcMotor intake;
    private CRServo leftIntakeServo;
    private CRServo rightIntakeServo;

    public IntakeSubsystem() {
        this(true);
    }

    public IntakeSubsystem(boolean servosEnabled) {
        this.servosEnabled = servosEnabled;
    }

    public void init(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (servosEnabled) {
            leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
            rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");
            rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
            leftIntakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        setPower(0);
    }

    public void setPower(double power) {
        power = Math.max(-1, Math.min(1, power));
        intake.setPower(power);
        if (servosEnabled) {
            leftIntakeServo.setPower(power);
            rightIntakeServo.setPower(power);
        }
    }

    public double getPower() {
        return intake.getPower();
    }

    @Override
    public void publishTelemetry(RobotTelemetry output) {
        output.number("Intake", getPower(), "power");
        output.value("Intake servos", servosEnabled ? "Enabled" : "Disabled");
    }
}
