package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.telemetry.RobotTelemetry;
import org.firstinspires.ftc.teamcode.telemetry.TelemetrySource;

public class IntakeSubsystem implements TelemetrySource {
    private final boolean servosEnabled;
    private final DcMotorSimple.Direction motorDirection;
    private final DcMotorSimple.Direction leftServoDirection;
    private final DcMotorSimple.Direction rightServoDirection;
    private DcMotor intake;
    private CRServo leftIntakeServo;
    private CRServo rightIntakeServo;

    public IntakeSubsystem() {
        this(true, DcMotorSimple.Direction.FORWARD);
    }

    public IntakeSubsystem(boolean servosEnabled) {
        this(servosEnabled, DcMotorSimple.Direction.FORWARD);
    }

    public IntakeSubsystem(boolean servosEnabled, DcMotorSimple.Direction motorDirection) {
        this(servosEnabled, motorDirection, DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.REVERSE);
    }

    public IntakeSubsystem(boolean servosEnabled, RobotProfile profile) {
        this(servosEnabled, profile.intakeMotorDirection, profile.intakeLeftServoDirection,
                profile.intakeRightServoDirection);
    }

    private IntakeSubsystem(boolean servosEnabled, DcMotorSimple.Direction motorDirection,
            DcMotorSimple.Direction leftServoDirection, DcMotorSimple.Direction rightServoDirection) {
        this.servosEnabled = servosEnabled;
        this.motorDirection = motorDirection;
        this.leftServoDirection = leftServoDirection;
        this.rightServoDirection = rightServoDirection;
    }

    public void init(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(motorDirection);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (servosEnabled) {
            leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
            rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");
            rightIntakeServo.setDirection(rightServoDirection);
            leftIntakeServo.setDirection(leftServoDirection);
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
