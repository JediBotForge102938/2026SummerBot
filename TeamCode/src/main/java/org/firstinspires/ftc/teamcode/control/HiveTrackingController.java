package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.teamcode.BioBuzzLimelightClusterAdapter.Alliance;
import org.firstinspires.ftc.teamcode.BioBuzzLimelightClusterAdapter.ClusterDetection;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.telemetry.RobotTelemetry;
import org.firstinspires.ftc.teamcode.telemetry.TelemetrySource;

import java.util.List;

/** Owns target selection and rotation only; translation remains under driver control. */
public class HiveTrackingController implements TelemetrySource {
    private static final double TURN_GAIN = 0.020;
    private static final double MAX_TURN_POWER = 0.60;
    private final VisionSubsystem vision;
    private final DriveModePreferences preferences;
    private Alliance alliance;
    private boolean tracking;
    private boolean hasValidResult;
    private ClusterDetection target;
    private String confirmedScorableCluster;
    private double horizontalError;
    private double turnPower;

    public HiveTrackingController(VisionSubsystem vision, DriveModePreferences preferences) {
        this.vision = vision;
        this.preferences = preferences;
        alliance = preferences.isRedAlliance() ? Alliance.RED : Alliance.BLUE;
    }

    public void selectAlliance(boolean left, boolean right) {
        if (left == right) {
            return;
        }
        Alliance selected = left ? Alliance.RED : Alliance.BLUE;
        if (selected != alliance) {
            preferences.saveRedAlliance(selected == Alliance.RED);
            alliance = selected;
            confirmedScorableCluster = null;
            clearTarget();
        }
    }

    public double update(boolean requested) {
        tracking = requested;
        clearTarget();
        if (!requested) {
            confirmedScorableCluster = null;
            return 0;
        }
        VisionSubsystem.DetectionSnapshot snapshot = vision.update(alliance);
        hasValidResult = snapshot.hasValidResult;
        if (hasValidResult) {
            target = findNearestTrackableCluster(
                    snapshot.clusters, true, confirmedScorableCluster);
        }
        confirmedScorableCluster = target == null ? null : target.name;
        if (target != null) {
            double x = target.poseCameraSpace.getPosition().x;
            double z = target.poseCameraSpace.getPosition().z;
            horizontalError = Math.toDegrees(Math.atan2(x, z));
            turnPower = getTurnPower(horizontalError);
        }
        return turnPower;
    }

    public void stop() {
        tracking = false;
        confirmedScorableCluster = null;
        clearTarget();
    }

    public boolean hasTarget() {
        return target != null;
    }

    public Alliance getAlliance() {
        return alliance;
    }

    private void clearTarget() {
        target = null;
        hasValidResult = false;
        horizontalError = 0;
        turnPower = 0;
    }

    @Override
    public void publishTelemetry(RobotTelemetry output) {
        output.coloredValue("Tracking alliance", alliance,
                alliance == Alliance.RED ? RobotTelemetry.Color.RED : RobotTelemetry.Color.BLUE);
        output.value("Vision camera", VisionSubsystem.LIMELIGHT_NAME);
        output.value("Vision pipeline", VisionSubsystem.LIMELIGHT_PIPELINE);
        output.status("Hive tracking", !tracking ? "MANUAL"
                        : target != null ? "TRACKING SCORABLE HIVE"
                        : hasValidResult ? "NO SCORABLE HIVE" : "NO FRESH VALID VISION",
                !tracking ? RobotTelemetry.Level.INFO
                        : target != null ? RobotTelemetry.Level.SUCCESS : RobotTelemetry.Level.WARNING);
        if (target != null) {
            output.value("Hive target", target.name);
            output.value("Hive tags", target.visibleTagIdsLeftToRight);
            output.value("Hive confirmation", target.visibleTagIdsLeftToRight.size() == 1
                    ? "RETAINED (one visible tag)" : "CURRENT (two or more tags)");
            output.number("Hive range", Math.hypot(target.poseCameraSpace.getPosition().x,
                    target.poseCameraSpace.getPosition().z), "in");
            output.number("Hive horizontal error", horizontalError, "degrees");
        }
        output.number("Tracking rotation request", turnPower, "power");
        output.line("Hold A: aim at nearest scorable alliance hive; left stick drives at 35%");
        output.line("While aiming, translation yields to rotation; tracking rotation is not slowed");
        output.line("Continuous rumble while holding A: no trackable hive");
        output.line("D-pad left: RED | right: BLUE (saved automatically)");
    }

    public static ClusterDetection findNearestTrackableCluster(
            List<ClusterDetection> clusters, boolean scorableOnly) {
        return findNearestTrackableCluster(clusters, scorableOnly, null);
    }

    private static ClusterDetection findNearestTrackableCluster(
            List<ClusterDetection> clusters, boolean scorableOnly, String confirmedScorableCluster) {
        ClusterDetection nearest = null;
        double nearestRange = Double.POSITIVE_INFINITY;
        for (ClusterDetection cluster : clusters) {
            if (!cluster.hasPose()) {
                continue;
            }
            double x = cluster.poseCameraSpace.getPosition().x;
            double z = cluster.poseCameraSpace.getPosition().z;
            if (!Double.isFinite(x) || !Double.isFinite(z) || z <= 0
                    || (scorableOnly && !isScorableOrConfirmed(cluster, confirmedScorableCluster))) {
                continue;
            }
            double range = Math.hypot(x, z);
            if (range < nearestRange) {
                nearest = cluster;
                nearestRange = range;
            }
        }
        return nearest;
    }

    private static boolean isScorableOrConfirmed(
            ClusterDetection cluster, String confirmedScorableCluster) {
        if (VisionSubsystem.classifyCell(cluster) == VisionSubsystem.CellStatus.SCORABLE) {
            return true;
        }
        // One tag cannot establish scorability, but can maintain the same confirmed target.
        double roll = VisionSubsystem.getClusterRollDegrees(cluster);
        return cluster.name.equals(confirmedScorableCluster)
                && cluster.visibleTagIdsLeftToRight.size() == 1
                && Double.isFinite(roll) && roll < 0;
    }

    public static double getTurnPower(double horizontalAngleDegrees) {
        if (!Double.isFinite(horizontalAngleDegrees)) {
            throw new IllegalArgumentException("Tracking angle must be finite");
        }
        if (Math.abs(horizontalAngleDegrees) <= 2.0) {
            return 0;
        }
        double magnitude = Math.max(0.12,
                Math.min(MAX_TURN_POWER, Math.abs(horizontalAngleDegrees) * TURN_GAIN));
        // Preserve the tracking test's turn direction for the rear-facing Limelight.
        return Math.copySign(magnitude, horizontalAngleDegrees);
    }
}
