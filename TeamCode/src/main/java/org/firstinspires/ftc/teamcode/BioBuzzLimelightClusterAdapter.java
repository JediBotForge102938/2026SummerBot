package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public final class BioBuzzLimelightClusterAdapter {
    private static final int TAGS_PER_CLUSTER = 4;
    // Mirrors the BioBuzz cluster definitions in FTC SDK 12.0.0.
    private static final double[] TAG_X_INCHES = {-6.5, -2.75, 2.75, 6.5};
    private static final double TAG_Y_INCHES = 7.1874;
    private static final double TAG_Z_INCHES = -5.622;

    // Tag IDs and names mirror the FTC SDK 12.0.0 BioBuzz tag library.
    private static final ClusterSpec[] CLUSTERS = {
            new ClusterSpec("RED SCORING", 30, Alliance.RED),
            new ClusterSpec("RED AUDIENCE", 34, Alliance.RED),
            new ClusterSpec("BLUE AUDIENCE", 38, Alliance.BLUE),
            new ClusterSpec("BLUE SCORING", 42, Alliance.BLUE)
    };

    public enum Alliance {
        RED,
        BLUE,
        BOTH
    }

    /**
     * The pose is the cell-opening origin in Limelight camera space, in inches:
     * X points right, Y down, and Z forward.
     */
    public static final class ClusterDetection {
        public final String name;
        public final List<Integer> visibleTagIds;
        public final int percentClusterFound;
        public final Pose3D poseCameraSpace;

        private ClusterDetection(
                String name,
                List<Integer> visibleTagIds,
                int percentClusterFound,
                Pose3D poseCameraSpace) {
            this.name = name;
            this.visibleTagIds = Collections.unmodifiableList(new ArrayList<>(visibleTagIds));
            this.percentClusterFound = percentClusterFound;
            this.poseCameraSpace = poseCameraSpace;
        }

        public boolean hasPose() {
            return poseCameraSpace != null;
        }
    }

    public List<ClusterDetection> detect(LLResult result, Alliance alliance) {
        if (alliance == null) {
            throw new IllegalArgumentException("alliance must not be null");
        }
        if (result == null || !result.isValid()) {
            return Collections.emptyList();
        }

        Map<Integer, LLResultTypes.FiducialResult> resultsByTagId = new HashMap<>();
        for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
            if (fiducial != null) {
                resultsByTagId.put(fiducial.getFiducialId(), fiducial);
            }
        }

        List<ClusterDetection> clusters = new ArrayList<>();
        for (ClusterSpec cluster : CLUSTERS) {
            if (alliance != Alliance.BOTH && cluster.alliance != alliance) {
                continue;
            }

            List<Integer> visibleTagIds = new ArrayList<>();
            PoseAccumulator poseAccumulator = new PoseAccumulator();
            for (int memberIndex = 0; memberIndex < TAGS_PER_CLUSTER; memberIndex++) {
                int tagId = cluster.firstTagId + memberIndex;
                LLResultTypes.FiducialResult fiducial = resultsByTagId.get(tagId);
                if (fiducial == null) {
                    continue;
                }

                visibleTagIds.add(tagId);
                Pose3D tagPose = fiducial.getTargetPoseCameraSpace();
                if (hasUsablePose(tagPose)) {
                    poseAccumulator.add(tagPose, TAG_X_INCHES[memberIndex], TAG_Y_INCHES, TAG_Z_INCHES);
                }
            }

            if (!visibleTagIds.isEmpty()) {
                int percentFound = Math.round(100.0f * visibleTagIds.size() / TAGS_PER_CLUSTER);
                clusters.add(new ClusterDetection(
                        cluster.name,
                        visibleTagIds,
                        percentFound,
                        poseAccumulator.build()));
            }
        }
        return Collections.unmodifiableList(clusters);
    }

    private static boolean hasUsablePose(Pose3D pose) {
        if (pose == null) {
            return false;
        }

        Position position = pose.getPosition();
        return Double.isFinite(position.x)
                && Double.isFinite(position.y)
                && Double.isFinite(position.z)
                && (position.x != 0.0 || position.y != 0.0 || position.z != 0.0);
    }

    private static final class ClusterSpec {
        final String name;
        final int firstTagId;
        final Alliance alliance;

        ClusterSpec(String name, int firstTagId, Alliance alliance) {
            this.name = name;
            this.firstTagId = firstTagId;
            this.alliance = alliance;
        }
    }

    private static final class PoseAccumulator {
        private int count;
        private double x;
        private double y;
        private double z;
        private double yawSin;
        private double yawCos;
        private double pitchSin;
        private double pitchCos;
        private double rollSin;
        private double rollCos;
        private long acquisitionTime;

        void add(Pose3D tagPose, double tagX, double tagY, double tagZ) {
            Position tagPosition = tagPose.getPosition().toUnit(DistanceUnit.INCH);
            YawPitchRollAngles orientation = tagPose.getOrientation();
            double[] offset = rotateTagOffsetToCamera(-tagX, -tagY, -tagZ, orientation);

            x += tagPosition.x + offset[0];
            y += tagPosition.y + offset[1];
            z += tagPosition.z + offset[2];

            double yaw = orientation.getYaw(AngleUnit.RADIANS);
            double pitch = orientation.getPitch(AngleUnit.RADIANS);
            double roll = orientation.getRoll(AngleUnit.RADIANS);
            yawSin += Math.sin(yaw);
            yawCos += Math.cos(yaw);
            pitchSin += Math.sin(pitch);
            pitchCos += Math.cos(pitch);
            rollSin += Math.sin(roll);
            rollCos += Math.cos(roll);
            acquisitionTime = orientation.getAcquisitionTime();
            count++;
        }

        Pose3D build() {
            if (count == 0) {
                return null;
            }

            return new Pose3D(
                    new Position(
                            DistanceUnit.INCH,
                            x / count,
                            y / count,
                            z / count,
                            acquisitionTime),
                    new YawPitchRollAngles(
                            AngleUnit.RADIANS,
                            Math.atan2(yawSin, yawCos),
                            Math.atan2(pitchSin, pitchCos),
                            Math.atan2(rollSin, rollCos),
                            acquisitionTime));
        }

        private static double[] rotateTagOffsetToCamera(
                double x,
                double y,
                double z,
                YawPitchRollAngles orientation) {
            double yaw = orientation.getYaw(AngleUnit.RADIANS);
            double pitch = orientation.getPitch(AngleUnit.RADIANS);
            double roll = orientation.getRoll(AngleUnit.RADIANS);
            // Limelight's target pose uses the standard Rz(yaw) * Ry(pitch) * Rx(roll) rotation.
            double cosYaw = Math.cos(yaw);
            double sinYaw = Math.sin(yaw);
            double cosPitch = Math.cos(pitch);
            double sinPitch = Math.sin(pitch);
            double cosRoll = Math.cos(roll);
            double sinRoll = Math.sin(roll);

            return new double[] {
                    cosYaw * cosPitch * x
                            + (cosYaw * sinPitch * sinRoll - sinYaw * cosRoll) * y
                            + (cosYaw * sinPitch * cosRoll + sinYaw * sinRoll) * z,
                    sinYaw * cosPitch * x
                            + (sinYaw * sinPitch * sinRoll + cosYaw * cosRoll) * y
                            + (sinYaw * sinPitch * cosRoll - cosYaw * sinRoll) * z,
                    -sinPitch * x + cosPitch * sinRoll * y + cosPitch * cosRoll * z
            };
        }
    }
}
