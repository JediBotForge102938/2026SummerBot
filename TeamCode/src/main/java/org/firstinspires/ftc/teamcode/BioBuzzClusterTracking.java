package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import java.util.List;

@TeleOp(name = "BioBuzz Cluster Tracking", group = "Vision")
public class BioBuzzClusterTracking extends OpMode {
    private static final double MAX_TURN_POWER = 0.45;
    private static final double MIN_TURN_POWER = 0.12;
    private static final double TURN_POWER_PER_DEGREE = 0.012;
    private static final double TURN_DEADBAND_DEGREES = 2.0;
    private static final double FOLLOW_BACKWARD_POWER = -0.25;

    private final DriveSubsystem drive = new DriveSubsystem();
    private final VisionSubsystem vision = new VisionSubsystem();
    private BioBuzzLimelightClusterAdapter.Alliance selectedAlliance =
            BioBuzzLimelightClusterAdapter.Alliance.BLUE;
    private boolean driveInitialized;

    @Override
    public void init() {
        drive.init(hardwareMap);
        driveInitialized = true;
        vision.init(hardwareMap);
        telemetry.addData("Selected alliance", selectedAlliance);
        telemetry.addLine("OPTIONS: switch alliance");
        telemetry.addLine("Hold A: turn to face nearest cluster");
        telemetry.addLine("Hold B: turn and drive toward nearest cluster");
        telemetry.addData("Limelight", VisionSubsystem.LIMELIGHT_NAME);
        telemetry.addData("Pipeline", VisionSubsystem.LIMELIGHT_PIPELINE);
    }

    @Override
    public void loop() {
        if (gamepad1.optionsWasPressed()) {
            selectedAlliance = selectedAlliance == BioBuzzLimelightClusterAdapter.Alliance.BLUE
                    ? BioBuzzLimelightClusterAdapter.Alliance.RED
                    : BioBuzzLimelightClusterAdapter.Alliance.BLUE;
        }

        VisionSubsystem.DetectionSnapshot snapshot = vision.update(selectedAlliance);
        BioBuzzLimelightClusterAdapter.ClusterDetection target =
                findNearestTrackableCluster(snapshot.clusters);

        double forward = 0.0;
        double rotate = 0.0;
        String mode = "Idle";
        if (gamepad1.b) {
            mode = "Follow";
        } else if (gamepad1.a) {
            mode = "Track";
        }

        if (target == null) {
            drive.driveRobotRelative(0.0, 0.0, 0.0);
            telemetry.addData(
                    "Tracking",
                    snapshot.hasValidResult
                            ? "No selected-alliance cluster with a forward pose"
                            : "No valid Limelight result");
        } else {
            Pose3D pose = target.poseCameraSpace;
            double targetX = pose.getPosition().x;
            double targetZ = pose.getPosition().z;
            double horizontalAngleDegrees = Math.toDegrees(Math.atan2(targetX, targetZ));
            if (gamepad1.a || gamepad1.b) {
                // This rear-facing camera needs the opposite chassis turn from a front-facing camera.
                rotate = getTurnPower(horizontalAngleDegrees);
            }
            if (gamepad1.b) {
                forward = FOLLOW_BACKWARD_POWER;
            }

            drive.driveRobotRelative(forward, 0.0, rotate);
            telemetry.addData("Target", target.name);
            telemetry.addData("Visible tags", target.visibleTagIds);
            telemetry.addData("Tags left to right", target.visibleTagIdsLeftToRight);
            telemetry.addData("Target range", "%.1f in", Math.hypot(targetX, targetZ));
            telemetry.addData("Horizontal error", "%.1f deg", horizontalAngleDegrees);
            telemetry.addData("Turn power", "%.2f", rotate);
            telemetry.addData(
                    "Cell status",
                    "%s (ascending IDs and negative roll)",
                    VisionSubsystem.classifyCell(target));
            telemetry.addData(
                    "Cluster roll (diagnostic)",
                    "%.1f deg",
                    VisionSubsystem.getClusterRollDegrees(target));
        }

        telemetry.addData("Selected alliance", selectedAlliance);
        telemetry.addData("Mode", mode);
        telemetry.addLine("OPTIONS: switch alliance | Hold A: track | Hold B: follow");
        telemetry.update();
    }

    @Override
    public void stop() {
        if (driveInitialized) {
            drive.driveRobotRelative(0.0, 0.0, 0.0);
        }
        vision.stop();
    }

    private static BioBuzzLimelightClusterAdapter.ClusterDetection findNearestTrackableCluster(
            List<BioBuzzLimelightClusterAdapter.ClusterDetection> clusters) {
        BioBuzzLimelightClusterAdapter.ClusterDetection nearest = null;
        double nearestRangeSquared = Double.POSITIVE_INFINITY;
        for (BioBuzzLimelightClusterAdapter.ClusterDetection cluster : clusters) {
            if (!cluster.hasPose()) {
                continue;
            }

            Pose3D pose = cluster.poseCameraSpace;
            double x = pose.getPosition().x;
            double z = pose.getPosition().z;
            if (!Double.isFinite(x) || !Double.isFinite(z) || z <= 0.0) {
                continue;
            }

            double rangeSquared = x * x + z * z;
            if (rangeSquared < nearestRangeSquared) {
                nearest = cluster;
                nearestRangeSquared = rangeSquared;
            }
        }
        return nearest;
    }

    private static double getTurnPower(double horizontalAngleDegrees) {
        if (Math.abs(horizontalAngleDegrees) <= TURN_DEADBAND_DEGREES) {
            return 0.0;
        }

        double magnitude = Math.min(
                MAX_TURN_POWER,
                Math.abs(horizontalAngleDegrees) * TURN_POWER_PER_DEGREE);
        magnitude = Math.max(MIN_TURN_POWER, magnitude);
        return Math.copySign(magnitude, horizontalAngleDegrees);
    }
}
