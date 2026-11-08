package org.firstinspires.ftc.teamcode.control;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import java.util.Arrays;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.BioBuzzLimelightClusterAdapter;
import org.firstinspires.ftc.teamcode.BioBuzzLimelightClusterAdapter.Alliance;
import org.firstinspires.ftc.teamcode.BioBuzzLimelightClusterAdapter.ClusterDetection;
import org.firstinspires.ftc.teamcode.TestProfiles;
import org.firstinspires.ftc.teamcode.subsystems.FieldMap;
import org.junit.Test;

public class ApproximateVisionPositionTest {
    private ClusterDetection cluster(int first, double roll) {
        LLResult result = mock(LLResult.class);
        when(result.isValid()).thenReturn(true);
        LLResultTypes.FiducialResult firstTag = tag(first, -5, roll);
        LLResultTypes.FiducialResult secondTag = tag(first + 1, 5, roll);
        when(result.getFiducialResults()).thenReturn(Arrays.asList(firstTag, secondTag));
        return new BioBuzzLimelightClusterAdapter().detect(result, Alliance.BOTH).get(0);
    }

    private LLResultTypes.FiducialResult tag(int id, double x, double roll) {
        LLResultTypes.FiducialResult result = mock(LLResultTypes.FiducialResult.class);
        when(result.getFiducialId()).thenReturn(id);
        when(result.getTargetPoseCameraSpace()).thenReturn(new Pose3D(
                new Position(DistanceUnit.INCH, x, 3, 60, 0),
                new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, roll, 0)));
        return result;
    }

    @Test public void cameraOffsetPitchAndImuHeadingAreApplied() {
        ClusterDetection cell = cluster(34, -10);
        Position p = cell.poseCameraSpace.getPosition().toUnit(DistanceUnit.INCH);
        double forward = -(p.z * Math.cos(Math.toRadians(37))
                + p.y * Math.sin(Math.toRadians(37))) + (9.5 - 45.72 / 2) / 2.54;
        double right = -p.x;
        FieldMap.Pose atZero = ApproximateVisionPosition.estimate(cell, TestProfiles.starter(), 0);
        assertEquals(-12 - forward, atZero.x, 1e-8);
        assertEquals(-9.4 + right, atZero.y, 1e-8);
        FieldMap.Pose atNinety = ApproximateVisionPosition.estimate(cell, TestProfiles.starter(), 90);
        assertEquals(-12 - right, atNinety.x, 1e-8);
        assertEquals(-9.4 - forward, atNinety.y, 1e-8);
    }

    @Test public void matchesEachClusterToItsOwnLandmark() {
        int[] ids = {30, 34, 38, 42};
        for (int id : ids) {
            ClusterDetection cell = cluster(id, -10);
            FieldMap.Pose pose = ApproximateVisionPosition.estimate(cell, TestProfiles.starter(), 0);
            Position p = cell.poseCameraSpace.getPosition();
            assertEquals((id < 38 ? -12 : 12)
                    + p.z * Math.cos(Math.toRadians(37))
                    + p.y * Math.sin(Math.toRadians(37)) - (9.5 - 45.72 / 2) / 2.54,
                    pose.x, 1e-8);
            assertEquals((id == 34 || id == 38 ? -9.4 : 9.4) - p.x, pose.y, 1e-8);
        }
    }

    @Test public void rejectsNonScorableOrInvalidHeading() {
        assertNull(ApproximateVisionPosition.estimate(cluster(34, 10), TestProfiles.starter(), 0));
        assertNull(ApproximateVisionPosition.estimate(cluster(34, -10), TestProfiles.starter(), Double.NaN));
    }
}
