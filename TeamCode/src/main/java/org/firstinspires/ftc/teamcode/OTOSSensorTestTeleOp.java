package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotProfile;

@TeleOp(name = "OTOS Sensor Test", group = "2 - (Tests)")
public class OTOSSensorTestTeleOp extends OpMode {
    private static final double STICK_DEADBAND = 0.08;
    private final DriveSubsystem drive = new DriveSubsystem();
    private final SparkFunOTOS.Pose2D position = new SparkFunOTOS.Pose2D();
    private final SparkFunOTOS.Pose2D velocity = new SparkFunOTOS.Pose2D();
    private final SparkFunOTOS.Pose2D acceleration = new SparkFunOTOS.Pose2D();
    private SparkFunOTOS otos;
    private boolean driveInitialized;
    private int drivePowerPercent = 45;
    private boolean previousUp;
    private boolean previousDown;
    private boolean previousX;
    private boolean previousOptions;
    private boolean previousA;
    private boolean returning;
    private double returnStarted;
    private double settledSince = Double.NaN;
    private String driveStatus = "Manual";

    @Override
    public void init() {
        drivePowerPercent = 45;
        previousUp = false;
        previousDown = false;
        previousX = false;
        previousOptions = false;
        previousA = false;
        returning = false;
        settledSince = Double.NaN;
        driveStatus = "Manual";
        RobotProfile profile = RobotProfile.loadActive(null);
        drive.init(hardwareMap, profile);
        driveInitialized = true;
        stopDrive();
        otos = hardwareMap.get(SparkFunOTOS.class, profile.otosName);
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);
        otos.setOffset(new SparkFunOTOS.Pose2D(0, 0, 0));
        otos.setLinearScalar(profile.otosLinearScalar);
        otos.setAngularScalar(profile.otosAngularScalar);
        calibrateImu();
        otos.resetTracking();

        SparkFunOTOS.Version hardware = new SparkFunOTOS.Version();
        SparkFunOTOS.Version firmware = new SparkFunOTOS.Version();
        otos.getVersionInfo(hardware, firmware);
        telemetry.addData("OTOS hardware", "%d.%d", hardware.major, hardware.minor);
        telemetry.addData("OTOS firmware", "%d.%d", firmware.major, firmware.minor);
        otos.getPosVelAcc(position, velocity, acceleration);
        publishReadings();
    }

    @Override
    public void init_loop() {
        update(false);
    }

    @Override
    public void loop() {
        update(true);
    }

    @Override
    public void stop() {
        if (driveInitialized) {
            cancelReturn("Stopped");
        }
    }

    private void update(boolean active) {
        otos.getPosVelAcc(position, velocity, acceleration);
        boolean x = gamepad1.x;
        boolean y = gamepad1.y;
        boolean options = gamepad1.options || gamepad1.start;
        boolean a = gamepad1.a;
        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;
        if (up && !previousUp && !down) {
            drivePowerPercent = Math.min(100, drivePowerPercent + 5);
        } else if (down && !previousDown && !up) {
            drivePowerPercent = Math.max(10, drivePowerPercent - 5);
        }
        previousUp = up;
        previousDown = down;
        if (x && !previousX && !y) {
            cancelReturn("Calibration requested: release sticks and keep robot still");
            if (stationary()) {
                calibrateImu();
                driveStatus = "IMU calibrated";
            } else {
                driveStatus = "Calibration refused: stop moving, then press X again";
            }
        }
        if (options && !previousOptions && !y) {
            cancelReturn("Home reset to current sensor position and heading");
            otos.resetTracking();
        }
        SparkFunOTOS.Status status = otos.getStatus();
        boolean manualInput = Math.abs(gamepad1.left_stick_y) > STICK_DEADBAND
                || Math.abs(gamepad1.left_stick_x) > STICK_DEADBAND
                || Math.abs(gamepad1.right_stick_x) > STICK_DEADBAND;
        if (y) {
            cancelReturn("Stopped by Y");
        } else if (!active || x || options) {
            stopDrive();
        } else if (!validReadings() || status.errorPaa || status.errorLsm) {
            cancelReturn("Drive stopped: OTOS error or invalid readings");
        } else {
            if (gamepad1.b) {
                cancelReturn("Manual override");
            } else if (manualInput) {
                returning = false;
                settledSince = Double.NaN;
            } else if (a && !previousA) {
                if (returning) {
                    cancelReturn("Return cancelled");
                } else {
                    returning = true;
                    returnStarted = getRuntime();
                    settledSince = Double.NaN;
                    driveStatus = "Returning to home";
                }
            }
            if (returning) {
                if (status.warnTiltAngle || status.warnOpticalTracking) {
                    cancelReturn("Return stopped: OTOS tracking warning");
                } else if (getRuntime() - returnStarted >= 12.0) {
                    cancelReturn("Return timed out: check tracking and drive directions");
                } else {
                    returnHome();
                }
            } else if (manualInput && !gamepad1.b) {
                driveStatus = "Manual";
                drive.driveRobotRelative(deadband(-gamepad1.left_stick_y),
                        deadband(gamepad1.left_stick_x), deadband(gamepad1.right_stick_x),
                        drivePowerPercent / 100.0);
            } else {
                stopDrive();
            }
        }
        previousX = x;
        previousOptions = options;
        previousA = a;
        publishReadings();
    }

    private void returnHome() {
        double headingError = AngleUnit.normalizeDegrees(-position.h);
        boolean atHome = Math.hypot(position.x, position.y) <= 1.0
                && Math.abs(headingError) <= 3.0;
        if (atHome && stationary()) {
            stopDrive();
            if (Double.isNaN(settledSince)) {
                settledSince = getRuntime();
            } else if (getRuntime() - settledSince >= 0.2) {
                cancelReturn("Home reached (within 1 inch / 3 degrees)");
            }
            return;
        }
        settledSince = Double.NaN;
        double h = Math.toRadians(position.h);
        double cos = Math.cos(h);
        double sin = Math.sin(h);
        // Measured mounting: forward = -X, right = +Y, clockwise = negative heading.
        double fieldX = -0.12 * position.x - 0.02 * velocity.x;
        double fieldY = -0.12 * position.y - 0.02 * velocity.y;
        double forward = -fieldX * cos - fieldY * sin;
        double right = -fieldX * sin + fieldY * cos;
        double magnitude = Math.hypot(forward, right);
        if (magnitude > 0.8) {
            forward *= 0.8 / magnitude;
            right *= 0.8 / magnitude;
        }
        double clockwise = Math.max(-0.6,
                Math.min(0.6, -0.025 * headingError + 0.004 * velocity.h));
        drive.driveRobotRelative(forward, right, clockwise, drivePowerPercent / 100.0, true);
    }

    private boolean stationary() {
        return validReadings() && Math.hypot(velocity.x, velocity.y) <= 1.0
                && Math.abs(velocity.h) <= 5.0;
    }

    private boolean validReadings() {
        return Double.isFinite(position.x) && Double.isFinite(position.y)
                && Double.isFinite(position.h) && Double.isFinite(velocity.x)
                && Double.isFinite(velocity.y) && Double.isFinite(velocity.h);
    }

    private void cancelReturn(String reason) {
        returning = false;
        settledSince = Double.NaN;
        driveStatus = reason;
        stopDrive();
    }

    private void stopDrive() {
        drive.driveRobotRelative(0, 0, 0);
    }

    private static double deadband(double value) {
        return Math.abs(value) > STICK_DEADBAND ? value : 0;
    }

    private void calibrateImu() {
        telemetry.addLine("Calibrating OTOS IMU: keep the robot flat and completely still.");
        telemetry.update();
        if (!otos.calibrateImu()) {
            throw new IllegalStateException("OTOS IMU calibration failed. Check sensor connection.");
        }
    }

    private void publishReadings() {
        SparkFunOTOS.Status status = otos.getStatus();
        telemetry.addLine("Sticks: drive. A: return home / cancel. Y: stop. B or sticks: cancel.");
        telemetry.addLine("D-pad up / down: adjust drive and return speed by 5%.");
        telemetry.addData("Drive power limit", "%d%%", drivePowerPercent);
        telemetry.addLine("Options / Start: set home here. X: calibrate IMU while stationary.");
        telemetry.addLine("Direct return only: no field map or obstacle avoidance.");
        telemetry.addData("Drive status", driveStatus);
        telemetry.addData("Distance to home (in)", "%.2f", Math.hypot(position.x, position.y));
        telemetry.addData("Position (in)", "X %.2f, Y %.2f", position.x, position.y);
        telemetry.addData("Heading (deg)", "%.2f", position.h);
        telemetry.addData("Velocity (in/s)", "X %.2f, Y %.2f", velocity.x, velocity.y);
        telemetry.addData("Turn rate (deg/s)", "%.2f", velocity.h);
        telemetry.addData("Acceleration (in/s^2)", "X %.2f, Y %.2f",
                acceleration.x, acceleration.y);
        telemetry.addData("Angular acceleration (deg/s^2)", "%.2f", acceleration.h);
        telemetry.addData("Tilt warning", status.warnTiltAngle);
        telemetry.addData("Optical tracking warning", status.warnOpticalTracking);
        telemetry.addData("Optical sensor error", status.errorPaa);
        telemetry.addData("IMU error", status.errorLsm);
        telemetry.update();
    }
}
