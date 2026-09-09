package org.firstinspires.ftc.teamcode.teaching.session1;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "[S1-03] Tank Drive - Exercise", group = "Teaching S1")
public class S1_03_TankDriveExerciseTeleOp extends OpMode {
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
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        setBrake();
        telemetry.addLine("Tank exercise: use one stick for each side.");
    }

    @Override
    public void loop() {
        double forward = gamepad1.left_stick_y;
        double rotate = gamepad1.right_stick_x;
        double left = LESSON_SPEED * (forward + rotate);
        double right = LESSON_SPEED * (forward - rotate);
        setSide(left, leftFrontDrive, leftBackDrive);
        setSide(right, rightFrontDrive, rightBackDrive);
        telemetry.addData("Exercise", "left %.2f | right %.2f", left, right);
    }

    private void setSide(double power, DcMotor front, DcMotor rear) {
        front.setPower(power);
        rear.setPower(power);
    }

    private void setBrake() {
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
