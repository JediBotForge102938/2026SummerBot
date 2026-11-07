package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.teamcode.BioBuzzLimelightClusterAdapter.Alliance;
import org.firstinspires.ftc.teamcode.BioBuzzLimelightClusterAdapter.ClusterDetection;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.telemetry.RobotTelemetry;
import org.firstinspires.ftc.teamcode.telemetry.TelemetrySource;

import java.util.List;
import java.util.function.DoubleSupplier;

/** Owns target selection and rotation only; translation remains under driver control. */
public class HiveTrackingController implements TelemetrySource {
    private static final double TURN_GAIN = 0.020;
    private static final double MAX_TURN_POWER = 0.60;
    private final VisionSubsystem vision;
    private final DriveModePreferences preferences;
    private final ImuAimController aim;
    private Alliance alliance;
    private boolean configurationManaged;
    private boolean tracking;
    private boolean hasValidResult;
    private ClusterDetection target;
    private String confirmedScorableCluster;
    private double horizontalError;
    private double turnPower;

    public HiveTrackingController(VisionSubsystem vision, DriveModePreferences preferences,
            DoubleSupplier yawRate) {
        this(vision, preferences, new ImuAimController(yawRate));
    }

    HiveTrackingController(VisionSubsystem vision, DriveModePreferences preferences,
            ImuAimController aim) {
        this.vision = vision;
        this.preferences = preferences;
        this.aim = aim;
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

    public void setAlliance(Alliance selected) {
        if (selected == null) {
            throw new IllegalArgumentException("Tracking alliance is required");
        }
        alliance = selected;
        configurationManaged = true;
        confirmedScorableCluster = null;
        clearTarget();
    }

    public double update(boolean requested) {
        aim.sample();
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
            turnPower = aim.turn(horizontalError, snapshot.observationAgeMs);
            horizontalError = aim.getCorrectedAngle();
        }
        return turnPower;
    }

    public void stop() {
        aim.reset();
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

    public String getTrackingStatus() {
        return !tracking ? "MANUAL" : target != null ? "LOCKED IN"
                : hasValidResult ? "NOT SCORABLE" : "SEARCHING";
    }

    public RobotTelemetry.Level getTrackingStatusLevel() {
        return !tracking ? RobotTelemetry.Level.INFO
                : target != null ? RobotTelemetry.Level.SUCCESS
                : hasValidResult ? RobotTelemetry.Level.ERROR : RobotTelemetry.Level.WARNING;
    }

    public String getTargetName() {
        return target == null ? "None" : target.name;
    }

    public double getTargetRangeInches() {
        if (target == null) {
            throw new IllegalStateException("No hive target");
        }
        return Math.hypot(target.poseCameraSpace.getPosition().x,
                target.poseCameraSpace.getPosition().z);
    }

    public double getHorizontalErrorDegrees() {
        return horizontalError;
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
        output.value("Vision camera", vision.getCameraName());
        output.value("Vision pipeline", vision.getPipeline());
        output.status("Hive tracking", getTrackingStatus(),
                getTrackingStatusLevel());
        if (target != null) {
            output.value("Hive target", target.name);
            output.value("Hive tags", target.visibleTagIdsLeftToRight);
            output.value("Hive confirmation", target.visibleTagIdsLeftToRight.size() == 1
                    ? "RETAINED (one visible tag)" : "CURRENT (two or more tags)");
            output.number("Hive range", Math.hypot(target.poseCameraSpace.getPosition().x,
                    target.poseCameraSpace.getPosition().z), "in");
            output.number("Hive horizontal error", horizontalError, "degrees");
            output.status("Hive motion compensation",
                    aim.isCompensated() ? "IMU ROTATION" : "WARMING UP (VISION + RATE DAMPING)",
                    aim.isCompensated() ? RobotTelemetry.Level.INFO : RobotTelemetry.Level.WARNING);
        }
        output.number("Tracking rotation request", turnPower, "power");
        output.line("Hold A: aim at nearest scorable alliance hive; left stick drives at 35%");
        output.line("While aiming, translation yields to rotation; tracking rotation is not slowed");
        output.line("Continuous rumble while holding A: no trackable hive");
        if (!configurationManaged) {
            output.line("D-pad left: RED | right: BLUE (saved automatically)");
        }
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
