package org.firstinspires.ftc.teamcode.teaching.session1;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "[S1-02] Tank - Exercise", group = "Teaching S1")
public class S1_02_TankExerciseTeleOp extends OpMode {
    private static final double LESSON_SPEED = 0.5;
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;

    @Override
    public void init() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        setBrake();
        telemetry.addLine("4x4 tank ready: left stick drives, right stick turns.");
    }

    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double rotate = gamepad1.right_stick_x;
        double left = LESSON_SPEED * (forward + rotate);
        double right = LESSON_SPEED * (forward - rotate);
        leftFrontDrive.setPower(left);
        leftBackDrive.setPower(left);
        rightFrontDrive.setPower(right);
        rightBackDrive.setPower(right);
        telemetry.addData("Tank", "left %.2f  right %.2f", left, right);
    }

    private void setBrake() {
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
