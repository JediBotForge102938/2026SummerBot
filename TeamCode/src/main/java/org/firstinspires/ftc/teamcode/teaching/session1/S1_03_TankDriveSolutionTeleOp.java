package org.firstinspires.ftc.teamcode.teaching.session1;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "[S1-03] Tank Drive - Solution", group = "Teaching S1")
public class S1_03_TankDriveSolutionTeleOp extends OpMode {
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
        telemetry.addLine("Tank drive ready: left stick controls left wheels.");
    }

    @Override
    public void loop() {
        double left = -LESSON_SPEED * gamepad1.left_stick_y;
        double right = -LESSON_SPEED * gamepad1.right_stick_y;
        setSide(left, leftFrontDrive, leftBackDrive);
        setSide(right, rightFrontDrive, rightBackDrive);
        telemetry.addData("Tank", "left %.2f | right %.2f", left, right);
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
