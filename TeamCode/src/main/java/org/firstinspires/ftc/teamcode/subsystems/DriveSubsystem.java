package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.telemetry.RobotTelemetry;
import org.firstinspires.ftc.teamcode.telemetry.TelemetrySource;

public class DriveSubsystem implements TelemetrySource {
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;
    private IMU imu;
    private double headingOffsetDegrees;
    private double fieldForwardHeadingDegrees;

    private double leftFrontPower;
    private double rightFrontPower;
    private double leftBackPower;
    private double rightBackPower;

    public void init(HardwareMap hardwareMap) {
        init(hardwareMap, "imu", RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
    }

    public void init(HardwareMap hardwareMap, RobotProfile profile) {
        init(hardwareMap, profile.imuName, profile.imuLogo, profile.imuUsb);
    }

    private void init(HardwareMap hardwareMap, String imuName,
            RevHubOrientationOnRobot.LogoFacingDirection logo,
            RevHubOrientationOnRobot.UsbFacingDirection usb) {
        headingOffsetDegrees = 0;
        fieldForwardHeadingDegrees = 0;
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

        imu = hardwareMap.get(IMU.class, imuName);
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                logo, usb);
        if (!imu.initialize(new IMU.Parameters(orientationOnRobot))) {
            throw new IllegalStateException("Unable to initialize the configured drive IMU");
        }
    }

    public void resetHeading() {
        resetHeading(0);
    }

    public void resetHeading(double fieldHeadingDegrees) {
        resetHeading(fieldHeadingDegrees, 0);
    }

    public void resetHeading(double fieldHeadingDegrees, double fieldForwardDegrees) {
        if (!Double.isFinite(fieldHeadingDegrees) || !Double.isFinite(fieldForwardDegrees)) {
            throw new IllegalArgumentException("Field and forward headings must be finite");
        }
        imu.resetYaw();
        headingOffsetDegrees = fieldHeadingDegrees;
        fieldForwardHeadingDegrees = fieldForwardDegrees;
    }

    public void driveFieldRelative(double forward, double strafe, double rotate) {
        driveFieldRelative(forward, strafe, rotate, 1);
    }

    public void driveFieldRelative(double forward, double strafe, double rotate, double powerScale) {
        driveFieldRelative(forward, strafe, rotate, powerScale, false);
    }

    public void driveFieldRelative(double forward, double strafe, double rotate, double powerScale,
            boolean prioritizeRotation) {
        double theta = Math.atan2(forward, strafe);
        double radius = Math.hypot(strafe, forward);
        double heading = Math.toRadians(getHeadingDegrees() - fieldForwardHeadingDegrees);

        theta = AngleUnit.normalizeRadians(theta - heading);

        double robotForward = radius * Math.sin(theta);
        double robotStrafe = radius * Math.cos(theta);
        mecanumDrive(robotForward, robotStrafe, rotate, powerScale, prioritizeRotation);
    }

    public void driveRobotRelative(double forward, double strafe, double rotate) {
        driveRobotRelative(forward, strafe, rotate, 1);
    }

    public void driveRobotRelative(double forward, double strafe, double rotate, double powerScale) {
        driveRobotRelative(forward, strafe, rotate, powerScale, false);
    }

    public void driveRobotRelative(double forward, double strafe, double rotate, double powerScale,
            boolean prioritizeRotation) {
        mecanumDrive(forward, strafe, rotate, powerScale, prioritizeRotation);
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
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)
                + headingOffsetDegrees;
    }

    public double getYawRateDegreesPerSecond() {
        return imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate;
    }

    @Override
    public void publishTelemetry(RobotTelemetry output) {
        output.number("Heading", getHeadingDegrees(), "degrees");
        output.number("Drive front left", leftFrontPower, "power");
        output.number("Drive front right", rightFrontPower, "power");
        output.number("Drive back left", leftBackPower, "power");
        output.number("Drive back right", rightBackPower, "power");
    }

    private void mecanumDrive(double forward, double strafe, double rotate, double powerScale,
            boolean prioritizeRotation) {
        if (!Double.isFinite(powerScale) || powerScale < 0 || powerScale > 1) {
            throw new IllegalArgumentException("Drive power scale must be between 0 and 1");
        }
        // Match the assembled chassis's lateral direction after any field-to-robot transform.
        strafe = -strafe;
        if (prioritizeRotation) {
            if (!Double.isFinite(rotate) || Math.abs(rotate) > 1) {
                throw new IllegalArgumentException("Prioritized rotation must be between -1 and 1");
            }
            // Reserve wheel power for rotation before allocating translation; precision still applies.
            double translation = Math.abs(forward) + Math.abs(strafe);
            double available = 1 - Math.abs(rotate);
            if (translation > available) {
                double translationScale = available / translation;
                forward *= translationScale;
                strafe *= translationScale;
            }
        }
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

        leftFrontPower *= powerScale;
        rightFrontPower *= powerScale;
        leftBackPower *= powerScale;
        rightBackPower *= powerScale;

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }
}
