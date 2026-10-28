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
 * <p>The cell-up check assumes the Limelight optical axis is aligned with the robot's launcher
 * axis, with the camera's right/down/forward frame mounted level. FTC camera axes are +X right,
 * +Y forward, +Z up; Limelight camera axes are +X right, +Y down, +Z forward. The basis change
 * maps Limelight's +Z rotation axis to FTC's +Y, so an optical-axis rotation (Limelight yaw)
 * becomes FTC cluster roll. This deliberately does not use Limelight's Euler {@code roll}, which
 * is about camera +X.</p>
 */
public class VisionSubsystem {
    public static final String LIMELIGHT_NAME = "limelight";
    public static final int LIMELIGHT_PIPELINE = 0;

    public enum CellStatus {
        SCORABLE,
        NOT_SCORABLE,
        UNKNOWN_POSE
    }

    public static final class DetectionSnapshot {
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
        if (cluster == null || !cluster.hasPose()) {
            return CellStatus.UNKNOWN_POSE;
        }

        double ftcRollDegrees = getFtcRollDegrees(cluster);
        if (!Double.isFinite(ftcRollDegrees)) {
            return CellStatus.UNKNOWN_POSE;
        }
        return Math.abs(ftcRollDegrees) < 90.0
                ? CellStatus.SCORABLE
                : CellStatus.NOT_SCORABLE;
    }

    public static double getFtcRollDegrees(
            BioBuzzLimelightClusterAdapter.ClusterDetection cluster) {
        if (cluster == null || !cluster.hasPose()) {
            return Double.NaN;
        }

        // The frame conversion is FTC = [Limelight.x, Limelight.z, -Limelight.y].
        // Conjugating a Limelight Rz(yaw) rotation by that basis change gives FTC Ry(yaw).
        return cluster.poseCameraSpace.getOrientation().getYaw(AngleUnit.DEGREES);
    }
}
