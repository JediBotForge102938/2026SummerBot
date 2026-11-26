package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.io.File;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.BioBuzzLimelightClusterAdapter.Alliance;
import org.firstinspires.ftc.teamcode.BioBuzzLimelightClusterAdapter.ClusterDetection;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotProfile;
import org.junit.Test;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class BioBuzzLimelightClusterAdapterTest {
    @Test
    public void identifiesClusterFromOneVisibleTag() {
        LLResult result = mock(LLResult.class);
        LLResultTypes.FiducialResult tag = mock(LLResultTypes.FiducialResult.class);
        when(result.isValid()).thenReturn(true);
        when(result.getFiducialResults()).thenReturn(Collections.singletonList(tag));
        when(tag.getFiducialId()).thenReturn(38);
        when(tag.getTargetPoseCameraSpace()).thenReturn(new Pose3D(
                new Position(DistanceUnit.INCH, 12, 3, 100, 0),
                new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0)));

        List<ClusterDetection> clusters =
                new BioBuzzLimelightClusterAdapter().detect(result, Alliance.BLUE);

        assertEquals(1, clusters.size());
        ClusterDetection cluster = clusters.get(0);
        assertEquals("BLUE AUDIENCE", cluster.name);
        assertEquals(Collections.singletonList(38), cluster.visibleTagIds);
        assertEquals(Collections.singletonList(38), cluster.visibleTagIdsLeftToRight);
        assertEquals(25, cluster.percentClusterFound);
        assertTrue(cluster.hasPose());
    }

    @Test
    public void invertedObservationsMatchUprightPoseAndClassification() {
        for (double roll : new double[] {-10, 10}) {
            LLResult upright = result(tag(38, -20, 3, 100, 25, roll),
                    tag(39, -10, 3, 100, 25, roll));
            LLResult inverted = result(tag(38, 20, -3, 100, -155, roll),
                    tag(39, 10, -3, 100, -155, roll));
            BioBuzzLimelightClusterAdapter adapter = new BioBuzzLimelightClusterAdapter();
            ClusterDetection expected = adapter.detect(upright, Alliance.BLUE).get(0);
            ClusterDetection actual = adapter.detect(inverted, Alliance.BLUE, true).get(0);
            assertEquals(expected.visibleTagIdsLeftToRight, actual.visibleTagIdsLeftToRight);
            assertEquals(VisionSubsystem.classifyCell(expected, 65),
                    VisionSubsystem.classifyCell(actual, 65));
            assertEquals(VisionSubsystem.CellStatus.SCORABLE, VisionSubsystem.classifyCell(actual, 65));
            Position expectedPosition = expected.poseCameraSpace.getPosition();
            Position actualPosition = actual.poseCameraSpace.getPosition();
            assertEquals(expectedPosition.x, actualPosition.x, 1e-8);
            assertEquals(expectedPosition.y, actualPosition.y, 1e-8);
            assertEquals(expectedPosition.z, actualPosition.z, 1e-8);
            assertEquals(expected.poseCameraSpace.getOrientation().getYaw(AngleUnit.DEGREES),
                    actual.poseCameraSpace.getOrientation().getYaw(AngleUnit.DEGREES), 1e-8);
            assertEquals(VisionSubsystem.getClusterRollDegrees(expected),
                    VisionSubsystem.getClusterRollDegrees(actual), 1e-8);
            assertEquals(VisionSubsystem.CellStatus.NOT_SCORABLE,
                    VisionSubsystem.classifyCell(adapter.detect(inverted, Alliance.BLUE).get(0), 65));
        }
    }

    @Test
    public void fullMountRotationRecoversSameLevelRollWithoutChangingAimingPose() {
        for (double cellRoll : new double[] {-20, 35, 170, -170}) {
            for (double heading : new double[] {-40, 0, 40}) {
                double[][] level = multiply(rotation(2, heading),
                        multiply(rotation(1, 15), rotation(0, cellRoll)));
                for (double[] mount : new double[][] {{37, 0}, {44, 180}, {44, 190}}) {
                    double[][] raw = multiply(rotation(2, -mount[1]),
                            multiply(rotation(0, -mount[0]), level));
                    YawPitchRollAngles angles = angles(raw);
                    boolean inverted = mount[1] >= 180;
                    double x = inverted ? 20 : -20;
                    LLResult observation = result(posedTag(38, x, angles),
                            posedTag(39, x / 2, angles));
                    BioBuzzLimelightClusterAdapter adapter = new BioBuzzLimelightClusterAdapter();
                    ClusterDetection camera = adapter.detect(observation, Alliance.BLUE, inverted).get(0);
                    ClusterDetection compensated = adapter.detect(observation, Alliance.BLUE,
                            inverted, mount[0], mount[1]).get(0);
                    assertEquals(cellRoll, VisionSubsystem.getClusterRollDegrees(compensated), 1e-8);
                    assertEquals(camera.visibleTagIdsLeftToRight, compensated.visibleTagIdsLeftToRight);
                    assertEquals(camera.poseCameraSpace.getPosition().x,
                            compensated.poseCameraSpace.getPosition().x, 1e-8);
                    assertEquals(camera.poseCameraSpace.getPosition().y,
                            compensated.poseCameraSpace.getPosition().y, 1e-8);
                    assertEquals(camera.poseCameraSpace.getPosition().z,
                            compensated.poseCameraSpace.getPosition().z, 1e-8);
                    assertEquals(VisionSubsystem.getCameraSpaceRollDegrees(camera),
                            VisionSubsystem.getCameraSpaceRollDegrees(compensated), 1e-8);
                    assertEquals(level[1][2], compensated.openingNormalUp, 1e-8);
                    assertEquals(cellRoll < 65 ? VisionSubsystem.CellStatus.SCORABLE
                            : VisionSubsystem.CellStatus.NOT_SCORABLE,
                            VisionSubsystem.classifyCell(compensated, 65));
                }
            }
        }
    }

    @Test
    public void visionUsesEachActiveProfilesMountingAngles() {
        for (String robot : new String[] {"starterbot", "competitionbot"}) {
            RobotProfile profile = RobotProfile.load(new File("../robot-configs"),
                    "biobuzz-" + robot, robot);
            double[][] raw = multiply(rotation(2, -profile.limelightRollDegrees),
                    multiply(rotation(0, -profile.limelightPitchUpDegrees), rotation(0, -20)));
            double x = profile.limelightInverted ? 20 : -20;
            LLResult observation = result(posedTag(38, x, angles(raw)),
                    posedTag(39, x / 2, angles(raw)));
            Limelight3A camera = mock(Limelight3A.class);
            HardwareMap hardware = mock(HardwareMap.class);
            when(hardware.get(Limelight3A.class, profile.limelightName)).thenReturn(camera);
            when(camera.getLatestResult()).thenReturn(observation);
            VisionSubsystem vision = new VisionSubsystem();
            vision.init(hardware, profile);
            verify(camera).pipelineSwitch(profile.limelightPipeline);
            ClusterDetection cell = vision.update(Alliance.BLUE).clusters.get(0);
            assertEquals(-20, VisionSubsystem.getClusterRollDegrees(cell), 1e-8);
            assertEquals(VisionSubsystem.CellStatus.SCORABLE,
                    VisionSubsystem.classifyCell(cell, profile.limelightScorableRollThresholdDegrees));
            vision.stop();
        }
    }

    @Test
    public void competitionReadingsUseTheNewReferenceNotTheOldZeroCrossing() {
        BioBuzzLimelightClusterAdapter adapter = new BioBuzzLimelightClusterAdapter();
        for (double rawRoll : new double[] {-18, 4, 45}) {
            YawPitchRollAngles orientation = new YawPitchRollAngles(
                    AngleUnit.DEGREES, 180, 0, rawRoll, 0);
            ClusterDetection cell = adapter.detect(result(
                    posedTag(38, 20, orientation), posedTag(39, 10, orientation)),
                    Alliance.BLUE, true, 44, 180).get(0);
            assertEquals(rawRoll, VisionSubsystem.getCameraSpaceRollDegrees(cell), 1e-8);
            assertEquals(rawRoll + 44, VisionSubsystem.getClusterRollDegrees(cell), 1e-8);
            assertEquals(rawRoll + 44 < 65 ? VisionSubsystem.CellStatus.SCORABLE
                    : VisionSubsystem.CellStatus.NOT_SCORABLE, VisionSubsystem.classifyCell(cell, 65));
        }
    }

    @Test
    public void compensatesBeforeAveragingAndRejectsIndeterminateRoll() {
        BioBuzzLimelightClusterAdapter adapter = new BioBuzzLimelightClusterAdapter();
        LLResult observation = result(
                posedTag(38, -20, angles(multiply(rotation(0, -44), rotation(0, 179)))),
                posedTag(39, -10, angles(multiply(rotation(0, -44), rotation(0, -179)))));
        ClusterDetection wrap = adapter.detect(observation, Alliance.BLUE, false, 44, 0).get(0);
        assertEquals(180, Math.abs(VisionSubsystem.getClusterRollDegrees(wrap)), 1e-8);
        LLResult singular = result(
                posedTag(38, -20, angles(rotation(1, 90))),
                posedTag(39, -10, angles(rotation(1, 90))));
        assertEquals(VisionSubsystem.CellStatus.UNKNOWN_POSE,
                VisionSubsystem.classifyCell(adapter.detect(singular, Alliance.BLUE).get(0), 65));
        LLResult invalid = result(posedTag(38, -20,
                new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, Double.NaN, 0)));
        assertFalse(adapter.detect(invalid, Alliance.BLUE).get(0).hasPose());
        assertThrows(IllegalArgumentException.class,
                () -> adapter.detect(observation, Alliance.BLUE, false, Double.NaN, 0));
    }

    @Test
    public void reportsRollsForEachPosedTagBeforeAveraging() {
        LLResultTypes.FiducialResult noPose = mock(LLResultTypes.FiducialResult.class);
        when(noPose.getFiducialId()).thenReturn(40);
        LLResult observation = result(
                posedTag(39, 10, new YawPitchRollAngles(AngleUnit.DEGREES, 180, 0, 5, 0)),
                noPose,
                posedTag(38, 20, new YawPitchRollAngles(AngleUnit.DEGREES, 180, 0, -15, 0)));
        ClusterDetection cluster = new BioBuzzLimelightClusterAdapter()
                .detect(observation, Alliance.BLUE, true, 37, 180).get(0);

        assertEquals(Arrays.asList(38, 39, 40), cluster.visibleTagIds);
        assertEquals(Arrays.asList(38, 39), cluster.visibleTagIdsLeftToRight);
        assertEquals(2, cluster.tagRolls.size());
        assertEquals(38, cluster.tagRolls.get(0).id);
        assertEquals(-15, cluster.tagRolls.get(0).cameraDegrees, 1e-8);
        assertEquals(22, cluster.tagRolls.get(0).mountCompensatedDegrees, 1e-8);
        assertEquals(39, cluster.tagRolls.get(1).id);
        assertEquals(5, cluster.tagRolls.get(1).cameraDegrees, 1e-8);
        assertEquals(42, cluster.tagRolls.get(1).mountCompensatedDegrees, 1e-8);
        assertEquals(32, cluster.mountCompensatedRollDegrees, 1e-8);
        assertThrows(UnsupportedOperationException.class, () -> cluster.tagRolls.clear());
    }

    private static LLResultTypes.FiducialResult posedTag(int id, double x, YawPitchRollAngles angles) {
        LLResultTypes.FiducialResult tag = mock(LLResultTypes.FiducialResult.class);
        when(tag.getFiducialId()).thenReturn(id);
        when(tag.getTargetPoseCameraSpace()).thenReturn(new Pose3D(
                new Position(DistanceUnit.INCH, x, 3, 100, 0), angles));
        return tag;
    }

    @Test
    public void openingNormalSignAndPerTagRangeSurviveMountingCompensation() {
        BioBuzzLimelightClusterAdapter adapter = new BioBuzzLimelightClusterAdapter();
        for (double[] mount : new double[][] {{37, 0}, {44, 180}, {44, 190}}) {
            for (double roll : new double[] {-60, 60}) {
                double[][] level = multiply(rotation(1, 35), rotation(0, roll));
                double[][] raw = multiply(rotation(2, -mount[1]),
                        multiply(rotation(0, -mount[0]), level));
                ClusterDetection cell = adapter.detect(result(posedTag(38, -20, angles(raw))),
                        Alliance.BLUE, mount[1] >= 180, mount[0], mount[1]).get(0);
                double expectedUp = -Math.sin(Math.toRadians(roll));
                assertEquals(expectedUp, cell.openingNormalUp, 1e-8);
                assertEquals(expectedUp, cell.minOpeningNormalUp, 1e-8);
                assertEquals(expectedUp, cell.maxOpeningNormalUp, 1e-8);
            }
        }
        ClusterDetection disagreement = adapter.detect(result(
                posedTag(38, -20, angles(rotation(0, -60))),
                posedTag(39, -10, angles(rotation(0, 60)))), Alliance.BLUE).get(0);
        assertEquals(0, disagreement.openingNormalUp, 1e-8);
        assertEquals(-Math.sqrt(3) / 2, disagreement.minOpeningNormalUp, 1e-8);
        assertEquals(Math.sqrt(3) / 2, disagreement.maxOpeningNormalUp, 1e-8);
    }

    @Test
    public void rollGateUsesConfiguredThresholdAndRequiresReliableTagOrder() {
        BioBuzzLimelightClusterAdapter adapter = new BioBuzzLimelightClusterAdapter();
        ClusterDetection raised = adapter.detect(result(
                posedTag(38, -20, angles(rotation(0, 49))),
                posedTag(39, -10, angles(rotation(0, 49)))), Alliance.BLUE).get(0);
        ClusterDetection tipped = adapter.detect(result(
                posedTag(38, -20, angles(rotation(0, 105))),
                posedTag(39, -10, angles(rotation(0, 105)))), Alliance.BLUE).get(0);
        assertEquals(VisionSubsystem.CellStatus.SCORABLE, VisionSubsystem.classifyCell(raised, 65));
        assertEquals(VisionSubsystem.CellStatus.NOT_SCORABLE, VisionSubsystem.classifyCell(tipped, 65));
        assertEquals(VisionSubsystem.CellStatus.NOT_SCORABLE,
                VisionSubsystem.classifyCell(raised, raised.mountCompensatedRollDegrees));
        assertEquals(VisionSubsystem.CellStatus.SCORABLE,
                VisionSubsystem.classifyCell(raised, raised.mountCompensatedRollDegrees + 0.01));
        assertEquals(VisionSubsystem.CellStatus.NOT_SCORABLE, VisionSubsystem.classifyCell(raised, 45));
        ClusterDetection oneTag = adapter.detect(result(
                posedTag(38, -20, angles(rotation(0, 20)))), Alliance.BLUE).get(0);
        assertEquals(VisionSubsystem.CellStatus.UNKNOWN_POSE, VisionSubsystem.classifyCell(oneTag, 65));
        ClusterDetection reversed = adapter.detect(result(
                posedTag(38, -10, angles(rotation(0, 20))),
                posedTag(39, -20, angles(rotation(0, 20)))), Alliance.BLUE).get(0);
        assertEquals(VisionSubsystem.CellStatus.NOT_SCORABLE, VisionSubsystem.classifyCell(reversed, 65));
        ClusterDetection mixed = adapter.detect(result(
                posedTag(38, -30, angles(rotation(0, 20))),
                posedTag(39, -10, angles(rotation(0, 20))),
                posedTag(40, -20, angles(rotation(0, 20)))), Alliance.BLUE).get(0);
        assertEquals(VisionSubsystem.CellStatus.UNKNOWN_POSE, VisionSubsystem.classifyCell(mixed, 65));
    }

    private static double[][] rotation(int axis, double degrees) {
        double c = Math.cos(Math.toRadians(degrees));
        double s = Math.sin(Math.toRadians(degrees));
        if (axis == 0) {
            return new double[][] {{1, 0, 0}, {0, c, -s}, {0, s, c}};
        }
        if (axis == 1) {
            return new double[][] {{c, 0, s}, {0, 1, 0}, {-s, 0, c}};
        }
        return new double[][] {{c, -s, 0}, {s, c, 0}, {0, 0, 1}};
    }

    private static double[][] multiply(double[][] a, double[][] b) {
        double[][] product = new double[3][3];
        for (int row = 0; row < 3; row++) {
            for (int column = 0; column < 3; column++) {
                for (int k = 0; k < 3; k++) {
                    product[row][column] += a[row][k] * b[k][column];
                }
            }
        }
        return product;
    }

    private static YawPitchRollAngles angles(double[][] matrix) {
        return new YawPitchRollAngles(AngleUnit.RADIANS,
                Math.atan2(matrix[1][0], matrix[0][0]),
                Math.atan2(-matrix[2][0], Math.hypot(matrix[2][1], matrix[2][2])),
                Math.atan2(matrix[2][1], matrix[2][2]), 0);
    }

    private static LLResult result(LLResultTypes.FiducialResult... tags) {
        LLResult result = mock(LLResult.class);
        when(result.isValid()).thenReturn(true);
        when(result.getFiducialResults()).thenReturn(Arrays.asList(tags));
        return result;
    }

    private static LLResultTypes.FiducialResult tag(
            int id, double x, double y, double z, double yaw, double roll) {
        LLResultTypes.FiducialResult tag = mock(LLResultTypes.FiducialResult.class);
        when(tag.getFiducialId()).thenReturn(id);
        when(tag.getTargetPoseCameraSpace()).thenReturn(new Pose3D(
                new Position(DistanceUnit.INCH, x, y, z, 0),
                new YawPitchRollAngles(AngleUnit.DEGREES, yaw, 12, roll, 0)));
        return tag;
    }
}
