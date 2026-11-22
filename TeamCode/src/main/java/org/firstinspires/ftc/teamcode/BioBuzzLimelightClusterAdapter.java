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
import java.util.Comparator;
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
     * The pose is the cell-opening origin in camera space, in inches, normalized
     * for an inverted mounting: X points right, Y down, and Z forward.
     */
    public static final class ClusterDetection {
        public final String name;
        public final Alliance alliance;
        public final boolean audienceSide;
        public final List<Integer> visibleTagIds;
        public final List<Integer> visibleTagIdsLeftToRight;
        public final int percentClusterFound;
        public final Pose3D poseCameraSpace;
        /** Tag roll in a level frame aligned with the camera heading; robot assumed level. */
        public final double mountCompensatedRollDegrees;
        /** Assumes tag +Z faces outward from the bottom; +1 means the opening faces straight up. */
        public final double openingNormalUp;
        public final double minOpeningNormalUp;
        public final double maxOpeningNormalUp;

        private ClusterDetection(
                String name,
                Alliance alliance,
                boolean audienceSide,
                List<Integer> visibleTagIds,
                List<Integer> visibleTagIdsLeftToRight,
                int percentClusterFound,
                Pose3D poseCameraSpace,
                double mountCompensatedRollDegrees,
                double openingNormalUp,
                double minOpeningNormalUp,
                double maxOpeningNormalUp) {
            this.name = name;
            this.alliance = alliance;
            this.audienceSide = audienceSide;
            this.visibleTagIds = Collections.unmodifiableList(new ArrayList<>(visibleTagIds));
            this.visibleTagIdsLeftToRight =
                    Collections.unmodifiableList(new ArrayList<>(visibleTagIdsLeftToRight));
            this.percentClusterFound = percentClusterFound;
            this.poseCameraSpace = poseCameraSpace;
            this.mountCompensatedRollDegrees = mountCompensatedRollDegrees;
            this.openingNormalUp = openingNormalUp;
            this.minOpeningNormalUp = minOpeningNormalUp;
            this.maxOpeningNormalUp = maxOpeningNormalUp;
        }

        public boolean hasPose() {
            return poseCameraSpace != null;
        }
    }

    public List<ClusterDetection> detect(LLResult result, Alliance alliance) {
        return detect(result, alliance, false);
    }

    public List<ClusterDetection> detect(LLResult result, Alliance alliance, boolean inverted) {
        return detect(result, alliance, inverted, 0, inverted ? 180 : 0);
    }

    public List<ClusterDetection> detect(LLResult result, Alliance alliance, boolean inverted,
            double pitchUpDegrees, double mountingRollDegrees) {
        if (alliance == null) {
            throw new IllegalArgumentException("alliance must not be null");
        }
        if (!Double.isFinite(pitchUpDegrees) || !Double.isFinite(mountingRollDegrees)) {
            throw new IllegalArgumentException("Camera mounting angles must be finite");
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
            List<TagObservation> posedTags = new ArrayList<>();
            PoseAccumulator poseAccumulator = new PoseAccumulator(inverted, pitchUpDegrees,
                    mountingRollDegrees);
            for (int memberIndex = 0; memberIndex < TAGS_PER_CLUSTER; memberIndex++) {
                int tagId = cluster.firstTagId + memberIndex;
                LLResultTypes.FiducialResult fiducial = resultsByTagId.get(tagId);
                if (fiducial == null) {
                    continue;
                }

                visibleTagIds.add(tagId);
                Pose3D tagPose = fiducial.getTargetPoseCameraSpace();
                if (hasUsablePose(tagPose)) {
                    Position tagPosition = tagPose.getPosition().toUnit(DistanceUnit.INCH);
                    posedTags.add(new TagObservation(tagId, inverted ? -tagPosition.x : tagPosition.x));
                    poseAccumulator.add(tagPose, TAG_X_INCHES[memberIndex], TAG_Y_INCHES, TAG_Z_INCHES);
                }
            }

            if (!visibleTagIds.isEmpty()) {
                posedTags.sort(Comparator.comparingDouble(tag -> tag.cameraX));
                List<Integer> visibleTagIdsLeftToRight = new ArrayList<>();
                for (TagObservation tag : posedTags) {
                    visibleTagIdsLeftToRight.add(tag.id);
                }
                int percentFound = Math.round(100.0f * visibleTagIds.size() / TAGS_PER_CLUSTER);
                clusters.add(new ClusterDetection(
                        cluster.name,
                        cluster.alliance,
                        cluster.firstTagId == 34 || cluster.firstTagId == 38,
                        visibleTagIds,
                        visibleTagIdsLeftToRight,
                        percentFound,
                        normalizePose(poseAccumulator.build(), inverted),
                        poseAccumulator.buildCompensatedRollDegrees(),
                        poseAccumulator.buildOpeningNormalUp(),
                        poseAccumulator.count == 0 ? Double.NaN : poseAccumulator.minOpeningNormalUp,
                        poseAccumulator.count == 0 ? Double.NaN : poseAccumulator.maxOpeningNormalUp));
            }
        }
        return Collections.unmodifiableList(clusters);
    }

    private static Pose3D normalizePose(Pose3D pose, boolean inverted) {
        if (pose == null || !inverted) {
            return pose;
        }
        Position position = pose.getPosition();
        YawPitchRollAngles orientation = pose.getOrientation();
        double yaw = orientation.getYaw(AngleUnit.RADIANS) + Math.PI;
        return new Pose3D(
                new Position(position.unit, -position.x, -position.y, position.z,
                        position.acquisitionTime),
                new YawPitchRollAngles(AngleUnit.RADIANS,
                        Math.atan2(Math.sin(yaw), Math.cos(yaw)),
                        orientation.getPitch(AngleUnit.RADIANS),
                        orientation.getRoll(AngleUnit.RADIANS),
                        orientation.getAcquisitionTime()));
    }

    private static boolean hasUsablePose(Pose3D pose) {
        if (pose == null) {
            return false;
        }

        Position position = pose.getPosition();
        YawPitchRollAngles orientation = pose.getOrientation();
        return Double.isFinite(position.x)
                && Double.isFinite(position.y)
                && Double.isFinite(position.z)
                && Double.isFinite(orientation.getYaw(AngleUnit.RADIANS))
                && Double.isFinite(orientation.getPitch(AngleUnit.RADIANS))
                && Double.isFinite(orientation.getRoll(AngleUnit.RADIANS))
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

    private static final class TagObservation {
        final int id;
        final double cameraX;

        TagObservation(int id, double cameraX) {
            this.id = id;
            this.cameraX = cameraX;
        }
    }

    private static final class PoseAccumulator {
        private final boolean inverted;
        private final double mountingPitch;
        private final double residualMountingRoll;
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
        private double compensatedRollSin;
        private double compensatedRollCos;
        private double openingNormalUpSum;
        private double minOpeningNormalUp = Double.POSITIVE_INFINITY;
        private double maxOpeningNormalUp = Double.NEGATIVE_INFINITY;
        private long acquisitionTime;

        PoseAccumulator(boolean inverted, double pitchUpDegrees, double mountingRollDegrees) {
            this.inverted = inverted;
            mountingPitch = Math.toRadians(pitchUpDegrees);
            residualMountingRoll = Math.toRadians(mountingRollDegrees - (inverted ? 180 : 0));
        }

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
            YawPitchRollAngles normalized = normalizePose(tagPose, inverted).getOrientation();
            // Compose Rx(camera elevation) * Rz(residual lens roll) * R(tag).
            // Transform basis vectors rather than subtracting coupled Euler angles.
            double[] tagYAxis = rotateTagOffsetToCamera(0, 1, 0, normalized);
            double[] tagZAxis = rotateTagOffsetToCamera(0, 0, 1, normalized);
            double down = Math.sin(residualMountingRoll) * tagZAxis[0]
                    + Math.cos(residualMountingRoll) * tagZAxis[1];
            // Level Y points down. Negating the outward tag normal makes opening-up equal tag-down.
            double openingUp = Math.cos(mountingPitch) * down
                    - Math.sin(mountingPitch) * tagZAxis[2];
            openingNormalUpSum += openingUp;
            minOpeningNormalUp = Math.min(minOpeningNormalUp, openingUp);
            maxOpeningNormalUp = Math.max(maxOpeningNormalUp, openingUp);
            double levelZY = levelForwardComponent(tagYAxis);
            double levelZZ = levelForwardComponent(tagZAxis);
            double compensatedRoll = Math.hypot(levelZY, levelZZ) < 1e-8
                    ? Double.NaN : Math.atan2(levelZY, levelZZ);
            compensatedRollSin += Math.sin(compensatedRoll);
            compensatedRollCos += Math.cos(compensatedRoll);
            acquisitionTime = orientation.getAcquisitionTime();
            count++;
        }

        private double levelForwardComponent(double[] vector) {
            double down = Math.sin(residualMountingRoll) * vector[0]
                    + Math.cos(residualMountingRoll) * vector[1];
            return Math.sin(mountingPitch) * down + Math.cos(mountingPitch) * vector[2];
        }

        double buildCompensatedRollDegrees() {
            if (count == 0 || Math.hypot(compensatedRollSin, compensatedRollCos) < 1e-8) {
                return Double.NaN;
            }
            return Math.toDegrees(Math.atan2(compensatedRollSin, compensatedRollCos));
        }

        double buildOpeningNormalUp() {
            return count == 0 ? Double.NaN : openingNormalUpSum / count;
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
