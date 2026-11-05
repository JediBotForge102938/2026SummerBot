package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.BioBuzzLimelightClusterAdapter;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Limelight-based BioBuzz cluster detection.
 *
 * <p>The scorable check assumes the camera faces the cell in the same direction as the robot's
 * launcher. It compares member-tag IDs from left to right: ascending IDs are upright, while
 * descending IDs are upside down. A cell is scorable only when the IDs ascend and cluster roll
 * is negative. At least two posed tags are needed to determine orientation.</p>
 */
public class VisionSubsystem {
    public static final String LIMELIGHT_NAME = "limelight";
    public static final int LIMELIGHT_PIPELINE = 0;
    public static final long MAX_RESULT_AGE_MS = 250;

    public enum CellStatus {
        SCORABLE,
        NOT_SCORABLE,
        UNKNOWN_POSE
    }

    public static final class DetectionSnapshot {
        /** True only for a valid result no older than MAX_RESULT_AGE_MS. */
        public final boolean hasValidResult;
        public final List<BioBuzzLimelightClusterAdapter.ClusterDetection> clusters;

        private DetectionSnapshot(
                boolean hasValidResult,
                List<BioBuzzLimelightClusterAdapter.ClusterDetection> clusters) {
            this.hasValidResult = hasValidResult;
            this.clusters = Collections.unmodifiableList(new ArrayList<>(clusters));
        }
    }

    private final BioBuzzLimelightClusterAdapter clusterAdapter =
            new BioBuzzLimelightClusterAdapter();
    private Limelight3A limelight;
    private boolean started;

    public void init(HardwareMap hardwareMap) {
        if (limelight != null) {
            throw new IllegalStateException("VisionSubsystem has already been initialized");
        }

        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
        limelight.start();
        started = true;
    }

    public DetectionSnapshot update(BioBuzzLimelightClusterAdapter.Alliance alliance) {
        if (!started) {
            throw new IllegalStateException("VisionSubsystem must be initialized before use");
        }
        if (alliance == null) {
            throw new IllegalArgumentException("alliance must not be null");
        }

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return new DetectionSnapshot(false, Collections.emptyList());
        }
        long ageMs = result.getStaleness();
        if (ageMs < 0 || ageMs > MAX_RESULT_AGE_MS) {
            return new DetectionSnapshot(false, Collections.emptyList());
        }
        return new DetectionSnapshot(true, clusterAdapter.detect(result, alliance));
    }

    public void stop() {
        if (started) {
            limelight.stop();
            started = false;
        }
    }

    public static CellStatus classifyCell(
            BioBuzzLimelightClusterAdapter.ClusterDetection cluster) {
        if (!hasForwardPose(cluster)) {
            return CellStatus.UNKNOWN_POSE;
        }

        // The cluster's tags are defined left-to-right in ascending ID order.
        List<Integer> leftToRightIds = cluster.visibleTagIdsLeftToRight;
        if (leftToRightIds.size() < 2) {
            return CellStatus.UNKNOWN_POSE;
        }

        double clusterRollDegrees = getClusterRollDegrees(cluster);
        if (!Double.isFinite(clusterRollDegrees)) {
            return CellStatus.UNKNOWN_POSE;
        }

        boolean ascending = true;
        boolean descending = true;
        for (int index = 1; index < leftToRightIds.size(); index++) {
            int previousId = leftToRightIds.get(index - 1);
            int currentId = leftToRightIds.get(index);
            ascending &= previousId < currentId;
            descending &= previousId > currentId;
        }
        if (ascending) {
            return clusterRollDegrees < 0.0
                    ? CellStatus.SCORABLE
                    : CellStatus.NOT_SCORABLE;
        }
        if (descending) {
            return CellStatus.NOT_SCORABLE;
        }
        return CellStatus.UNKNOWN_POSE;
    }

    public static double getClusterRollDegrees(
            BioBuzzLimelightClusterAdapter.ClusterDetection cluster) {
        if (!hasForwardPose(cluster)) {
            return Double.NaN;
        }

        return cluster.poseCameraSpace.getOrientation().getRoll(AngleUnit.DEGREES);
    }

    private static boolean hasForwardPose(
            BioBuzzLimelightClusterAdapter.ClusterDetection cluster) {
        if (cluster == null || !cluster.hasPose()) {
            return false;
        }

        double forward = cluster.poseCameraSpace.getPosition().z;
        return Double.isFinite(forward) && forward > 0.0;
    }
}
