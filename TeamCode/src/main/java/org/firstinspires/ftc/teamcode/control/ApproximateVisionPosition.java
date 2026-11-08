package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.BioBuzzLimelightClusterAdapter.ClusterDetection;
import org.firstinspires.ftc.teamcode.subsystems.ConfigurationSubsystem.StartingSpot;
import org.firstinspires.ftc.teamcode.subsystems.FieldMap;
import org.firstinspires.ftc.teamcode.subsystems.RobotProfile;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/** Diagnostic estimate using the IMU heading and an approximate cell-opening location. */
public final class ApproximateVisionPosition {
    private ApproximateVisionPosition() { }

    /** Returns null when the detection or heading cannot support a provisional estimate. */
    public static FieldMap.Pose estimate(ClusterDetection cell, RobotProfile profile,
            double headingDegrees) {
        if (cell == null || profile == null || !Double.isFinite(headingDegrees)
                || VisionSubsystem.classifyCell(cell) != VisionSubsystem.CellStatus.SCORABLE) {
            return null;
        }
        Position p = cell.poseCameraSpace.getPosition().toUnit(DistanceUnit.INCH);
        if (!Double.isFinite(p.x) || !Double.isFinite(p.y) || !Double.isFinite(p.z)) {
            return null;
        }
        FieldMap.Pose landmark = FieldMap.hiveCell(cell.alliance,
                cell.audienceSide ? StartingSpot.AUDIENCE : StartingSpot.SCORING);
        double roll = Math.toRadians(profile.limelightRollDegrees);
        double right = p.x * Math.cos(roll) - p.y * Math.sin(roll);
        double down = p.x * Math.sin(roll) + p.y * Math.cos(roll);
        double pitch = Math.toRadians(profile.limelightPitchUpDegrees);
        double horizontal = p.z * Math.cos(pitch) + down * Math.sin(pitch);
        double yaw = Math.toRadians(profile.limelightYawDegrees);
        double forward = horizontal * Math.cos(yaw) + right * Math.sin(yaw)
                + (profile.limelightFromBackCm - profile.chassisLengthCm / 2) / 2.54;
        double robotRight = -horizontal * Math.sin(yaw) + right * Math.cos(yaw)
                + (profile.chassisWidthCm / 2 - profile.limelightFromLeftCm) / 2.54;
        double heading = Math.toRadians(headingDegrees);
        double fieldX = forward * Math.cos(heading) + robotRight * Math.sin(heading);
        double fieldY = forward * Math.sin(heading) - robotRight * Math.cos(heading);
        if (!Double.isFinite(fieldX) || !Double.isFinite(fieldY)) return null;
        return new FieldMap.Pose(landmark.x - fieldX, landmark.y - fieldY, headingDegrees);
    }
}
