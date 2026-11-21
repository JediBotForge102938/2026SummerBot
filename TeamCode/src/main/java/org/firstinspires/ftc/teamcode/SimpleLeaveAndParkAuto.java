package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FieldMap;
import org.firstinspires.ftc.teamcode.subsystems.RobotProfile;
import org.firstinspires.ftc.teamcode.subsystems.TrackingSubsystem;

@Autonomous(name = "Simple Leave & Park", group = "1 - (Autonomous)")
public final class SimpleLeaveAndParkAuto extends OpMode {
    private final DriveSubsystem drive = new DriveSubsystem();
    private final TrackingSubsystem tracking = new TrackingSubsystem();
    private RobotProfile profile;
    private final FieldMap.Pose initial = new FieldMap.Pose(0, 0, 90);
    private double distance;
    private double startedAt;
    private double progressAt;
    private double bestProgress;
    private boolean driveReady;
    private boolean active;
    private boolean started;
    private String status = "Not initialized";

    @Override public void init() {
        profile = RobotProfile.loadActive(null);
        drive.init(hardwareMap, profile);
        driveReady = true;
        stopDrive();
        tracking.init(hardwareMap, profile);
        distance = profile.chassisLengthCm / 2.54;
        status = "Ready: will drive forward one chassis length";
        publish();
    }

    @Override public void init_loop() {
        publish();
    }

    @Override public void start() {
        started = true;
        if (!profile.navigation.geometryVerified) {
            finish("Stopped: profile geometry not verified");
            publish();
            return;
        }
        drive.resetHeading(initial.headingDegrees);
        tracking.start(initial);
        tracking.update(drive.getHeadingDegrees(), drive.getYawRateDegreesPerSecond());
        startedAt = getRuntime();
        progressAt = startedAt;
        bestProgress = 0;
        active = true;
        status = "Driving straight";
        publish();
    }

    @Override public void loop() {
        if (active) {
            tracking.update(drive.getHeadingDegrees(), drive.getYawRateDegreesPerSecond());
            FieldMap.Pose pose = tracking.getPose();
            double now = getRuntime();
            if (!tracking.hasValidPose() || pose == null) {
                finish("Stopped: tracking unavailable or OTOS warning");
            } else if (!Double.isFinite(now) || now < progressAt
                    || now - startedAt >= profile.navigation.timeoutSeconds) {
                finish("Stopped: timeout");
            } else if (Math.abs(angleDifference(pose.headingDegrees, initial.headingDegrees))
                    > profile.navigation.headingToleranceDegrees
                    || Math.abs(pose.x - initial.x) > profile.navigation.arrivalToleranceIn) {
                finish("Stopped: heading or sideways drift");
            } else {
                double progress = forwardProgress(pose);
                if (progress >= distance) {
                    finish("Complete: moved forward one chassis length");
                } else if (progress > bestProgress + profile.navigation.progressThresholdIn) {
                    bestProgress = progress;
                    progressAt = now;
                    drive.driveRobotRelative(drivePower(), 0, 0);
                } else if (now - progressAt >= profile.navigation.progressTimeoutSeconds) {
                    finish("Stopped: no forward progress");
                } else {
                    drive.driveRobotRelative(drivePower(), 0, 0);
                }
            }
        } else {
            stopDrive();
        }
        publish();
    }

    @Override public void stop() { finish("OpMode stopped"); }

    private void finish(String reason) {
        active = false;
        status = reason;
        stopDrive();
    }

    private void stopDrive() {
        if (driveReady) drive.driveRobotRelative(0, 0, 0);
    }

    private double drivePower() {
        return profile.navigation.finalApproachPower;
    }

    private double forwardProgress(FieldMap.Pose pose) {
        return pose.y - initial.y;
    }

    private void publish() {
        telemetry.clearAll();
        telemetry.addLine("Simple Leave & Park");
        if (!started) {
            telemetry.addData("After START", "Moves forward one chassis length: %.1f in (%.1f cm)",
                    distance, profile.chassisLengthCm);
            telemetry.addLine("Face the parking target; place the bot one chassis length");
            telemetry.addLine("behind its desired final position. Keep flat and still in INIT.");
        }
        telemetry.addData("Robot profile", profile.name);
        telemetry.addData("Drive power", "%.0f%%", drivePower() * 100);
        telemetry.addData("Forward target", "%.1f in", distance);
        FieldMap.Pose pose = tracking.getPose();
        if (pose != null) {
            telemetry.addData("Forward progress", "%.1f in", forwardProgress(pose));
            telemetry.addData("Relative pose", "X %.1f Y %.1f H %.1f",
                    pose.x, pose.y, pose.headingDegrees);
        }
        telemetry.addLine(status);
        telemetry.addLine("Straight only; no obstacle avoidance. Driver Station STOP stops.");
        telemetry.update();
    }

    private static double angleDifference(double a, double b) {
        return a - 360 * Math.floor((a - b + 180) / 360) - b;
    }
}
