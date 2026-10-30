package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DriveSubsystem {
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;
    private IMU imu;

    private double leftFrontPower;
    private double rightFrontPower;
    private double leftBackPower;
    private double rightBackPower;

    public void init(HardwareMap hardwareMap) {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    public void resetHeading() {
        imu.resetYaw();
    }

    public void driveFieldRelative(double forward, double strafe, double rotate) {
        double theta = Math.atan2(forward, strafe);
        double radius = Math.hypot(strafe, forward);
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        theta = AngleUnit.normalizeRadians(theta - heading);

        double robotForward = radius * Math.sin(theta);
        double robotStrafe = radius * Math.cos(theta);
        mecanumDrive(robotForward, robotStrafe, rotate);
    }

    public void driveRobotRelative(double forward, double strafe, double rotate) {
        mecanumDrive(forward, strafe, rotate);
    }

    public double getLeftFrontPower() {
        return leftFrontPower;
    }

    public double getRightFrontPower() {
        return rightFrontPower;
    }

    public double getLeftBackPower() {
        return leftBackPower;
    }

    public double getRightBackPower() {
        return rightBackPower;
    }

    public double getHeadingDegrees() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    private void mecanumDrive(double forward, double strafe, double rotate) {
        leftFrontPower = forward + strafe + rotate;
        rightFrontPower = forward - strafe - rotate;
        leftBackPower = forward - strafe + rotate;
        rightBackPower = forward + strafe - rotate;

        double max = Math.max(
                Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),
                Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)));
        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }
}
