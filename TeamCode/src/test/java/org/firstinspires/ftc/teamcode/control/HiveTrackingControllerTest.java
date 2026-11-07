package org.firstinspires.ftc.teamcode.control;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.BioBuzzLimelightClusterAdapter;
import org.firstinspires.ftc.teamcode.BioBuzzLimelightClusterAdapter.Alliance;
import org.firstinspires.ftc.teamcode.BioBuzzLimelightClusterAdapter.ClusterDetection;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.telemetry.RobotTelemetry;
import org.junit.Before;
import org.junit.Test;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class HiveTrackingControllerTest {
    private Limelight3A camera;
    private VisionSubsystem vision;
    private DriveModePreferences preferences;
    private HiveTrackingController tracking;
    private List<LLResultTypes.FiducialResult> tags;
    private LLResult result;

    @Before
    public void setUp() {
        camera = mock(Limelight3A.class);
        HardwareMap hardware = mock(HardwareMap.class);
        when(hardware.get(Limelight3A.class, "limelight")).thenReturn(camera);
        result = mock(LLResult.class);
        tags = new ArrayList<>();
        when(result.isValid()).thenReturn(true);
        when(result.getStaleness()).thenReturn(0L);
        when(result.getFiducialResults()).thenReturn(tags);
        when(camera.getLatestResult()).thenReturn(result);
        vision = new VisionSubsystem();
        vision.init(hardware);
        preferences = mock(DriveModePreferences.class);
        tracking = new HiveTrackingController(vision, preferences, () -> 0);
    }

    @Test
    public void configuredAllianceFeedsVisionWithoutSavingPreferencesOrShowingSelectionControls() {
        addCluster(30, 10, 100, -10, false);
        addCluster(42, -10, 100, -10, false);
        clearInvocations(preferences);
        tracking.setAlliance(Alliance.RED);
        assertEquals(Alliance.RED, tracking.getAlliance());
        assertTrue(tracking.update(true) > 0);
        RobotTelemetry output = mock(RobotTelemetry.class);
        tracking.publishTelemetry(output);
        verify(output).value("Hive target", "RED SCORING");
        verify(output, never()).line("D-pad left: RED | right: BLUE (saved automatically)");
        verifyNoInteractions(preferences);
        assertThrows(IllegalArgumentException.class, () -> tracking.setAlliance(null));
    }

    @Test
    public void excludesNearNonScorableAndWrongAllianceAndTracksFarScorableHive() {
        addCluster(30, 5, 50, -10, false); // red, not the selected alliance
        addCluster(38, -20, 70, 10, false); // blue, wrong roll
        addCluster(42, 40, 200, -10, false); // blue and scorable

        assertTrue(tracking.update(true) > 0);

        RobotTelemetry output = mock(RobotTelemetry.class);
        tracking.publishTelemetry(output);
        verify(output).value("Hive target", "BLUE SCORING");
        verify(output).status("Hive tracking", "LOCKED IN", RobotTelemetry.Level.SUCCESS);
    }

    @Test
    public void nearestScorableAudienceHiveIsAllowed() {
        addCluster(38, -40, 100, -10, false);
        addCluster(42, 40, 200, -10, false);

        assertTrue(tracking.update(true) < 0);
        RobotTelemetry output = mock(RobotTelemetry.class);
        tracking.publishTelemetry(output);
        verify(output).value("Hive target", "BLUE AUDIENCE");
    }

    @Test
    public void allianceTelemetryUsesSelectedAllianceColor() {
        RobotTelemetry output = mock(RobotTelemetry.class);
        tracking.publishTelemetry(output);
        verify(output).coloredValue("Tracking alliance", Alliance.BLUE, RobotTelemetry.Color.BLUE);
        tracking.selectAlliance(true, false);
        tracking.publishTelemetry(output);
        verify(output).coloredValue("Tracking alliance", Alliance.RED, RobotTelemetry.Color.RED);
    }

    @Test
    public void targetPresenceIsIndependentOfTurnPowerAndClearedOnReleaseOrVisionLoss() {
        assertFalse(tracking.hasTarget());
        addCluster(38, -4.625, 100, -10, false);
        assertEquals(0, tracking.update(true), 0);
        assertTrue(tracking.hasTarget());
        tracking.update(false);
        assertFalse(tracking.hasTarget());
        tracking.update(true);
        assertTrue(tracking.hasTarget());
        when(result.isValid()).thenReturn(false);
        tracking.update(true);
        assertFalse(tracking.hasTarget());
    }

    @Test
    public void unknownSingleTagAndDescendingAndWrongRollAreRejected() {
        addTag(38, -10, 100, -10);
        addCluster(42, 10, 100, -10, true);
        addCluster(30, 10, 100, 10, false);

        assertEquals(0, tracking.update(true), 0);
        tracking.selectAlliance(true, false);
        assertEquals(0, tracking.update(true), 0);
    }

    @Test
    public void confirmedClusterContinuesWithOneTagButAnotherClusterCannotBorrowConfirmation() {
        addCluster(38, 40, 100, -10, false);
        assertTrue(tracking.update(true) > 0);
        tags.clear();
        addTag(39, 40, 100, -10);
        assertTrue(tracking.update(true) > 0);
        RobotTelemetry output = mock(RobotTelemetry.class);
        tracking.publishTelemetry(output);
        verify(output).value("Hive confirmation", "RETAINED (one visible tag)");

        tags.clear();
        addTag(42, 40, 100, -10);
        assertEquals(0, tracking.update(true), 0);
        tags.clear();
        addTag(39, 40, 100, -10);
        assertEquals(0, tracking.update(true), 0);
    }

    @Test
    public void freshContradictoryOrientationClearsConfirmation() {
        addCluster(38, 40, 100, -10, false);
        assertTrue(tracking.update(true) > 0);
        tags.clear();
        addTag(39, 40, 100, 10);
        assertEquals(0, tracking.update(true), 0);
        tags.clear();
        addTag(39, 40, 100, -10);
        assertEquals(0, tracking.update(true), 0);

        tags.clear();
        addCluster(38, 40, 100, -10, false);
        assertTrue(tracking.update(true) > 0);
        tags.clear();
        addCluster(38, 40, 100, -10, true);
        assertEquals(0, tracking.update(true), 0);
    }

    @Test
    public void lossReleaseStopAndAllianceChangesRequireTwoTagReconfirmation() {
        for (int reset = 0; reset < 7; reset++) {
            tags.clear();
            when(result.isValid()).thenReturn(true);
            when(result.getStaleness()).thenReturn(0L);
            when(camera.getLatestResult()).thenReturn(result);
            addCluster(38, 40, 100, -10, false);
            assertTrue(tracking.update(true) > 0);
            switch (reset) {
                case 0:
                    when(result.getStaleness()).thenReturn(251L);
                    assertEquals(0, tracking.update(true), 0);
                    when(result.getStaleness()).thenReturn(0L);
                    break;
                case 1:
                    when(result.isValid()).thenReturn(false);
                    assertEquals(0, tracking.update(true), 0);
                    when(result.isValid()).thenReturn(true);
                    break;
                case 2:
                    when(camera.getLatestResult()).thenReturn(null);
                    assertEquals(0, tracking.update(true), 0);
                    when(camera.getLatestResult()).thenReturn(result);
                    break;
                case 3:
                    tags.clear();
                    assertEquals(0, tracking.update(true), 0);
                    break;
                case 4:
                    tracking.update(false);
                    break;
                case 5:
                    tracking.stop();
                    break;
                case 6:
                    tracking.selectAlliance(true, false);
                    tracking.selectAlliance(false, true);
                    break;
                default:
                    fail("Unexpected reset case");
            }
            tags.clear();
            addTag(39, 40, 100, -10);
            assertEquals("reset case " + reset, 0, tracking.update(true), 0);
        }
    }

    @Test
    public void invalidMissingOrStaleFramesNeverReusePreviousTurn() {
        addCluster(38, 40, 100, -10, false);
        assertTrue(tracking.update(true) > 0);
        when(result.getStaleness()).thenReturn(251L);
        assertEquals(0, tracking.update(true), 0);
        when(result.getStaleness()).thenReturn(250L);
        assertTrue(tracking.update(true) > 0);
        when(result.isValid()).thenReturn(false);
        assertEquals(0, tracking.update(true), 0);
        when(camera.getLatestResult()).thenReturn(null);
        assertEquals(0, tracking.update(true), 0);
        RobotTelemetry output = mock(RobotTelemetry.class);
        tracking.publishTelemetry(output);
        verify(output, never()).value(eq("Hive target"), any());
        verify(output).status("Hive tracking", "SEARCHING", RobotTelemetry.Level.WARNING);
    }

    @Test
    public void releaseStopsPollingAndClearsTarget() {
        addCluster(38, 40, 100, -10, false);
        tracking.update(true);
        clearInvocations(camera);

        assertEquals(0, tracking.update(false), 0);
        verifyNoInteractions(camera);
        RobotTelemetry output = mock(RobotTelemetry.class);
        tracking.publishTelemetry(output);
        verify(output).status("Hive tracking", "MANUAL", RobotTelemetry.Level.INFO);
        verify(output, never()).value(eq("Hive target"), any());
    }

    @Test
    public void allianceSelectionSavesOnlyChangesAndSwitchesFiltering() {
        addCluster(30, -40, 100, -10, false);
        addCluster(38, 40, 100, -10, false);
        assertTrue(tracking.update(true) > 0);
        tracking.selectAlliance(true, false);
        tracking.selectAlliance(true, false);
        assertTrue(tracking.update(true) < 0);
        verify(preferences).saveRedAlliance(true);
        tracking.selectAlliance(true, true);
        tracking.selectAlliance(false, false);
        tracking.selectAlliance(false, true);
        assertTrue(tracking.update(true) > 0);
        verify(preferences).saveRedAlliance(false);
        verify(preferences, times(2)).saveRedAlliance(anyBoolean());
    }

    @Test
    public void restoresRedAndFailedSaveLeavesAllianceUnchanged() {
        addCluster(30, -40, 100, -10, false);
        addCluster(38, 40, 100, -10, false);
        when(preferences.isRedAlliance()).thenReturn(true);
        tracking = new HiveTrackingController(vision, preferences, () -> 0);
        assertTrue(tracking.update(true) < 0);
        doThrow(new IllegalStateException("Cannot save")).when(preferences).saveRedAlliance(false);
        assertThrows(IllegalStateException.class, () -> tracking.selectAlliance(false, true));
        assertTrue(tracking.update(true) < 0);
    }

    @Test
    public void noPoseBehindCameraAndNonFiniteTargetsAreNotTrackable() {
        ClusterDetection cluster = mock(ClusterDetection.class);
        when(cluster.hasPose()).thenReturn(false);
        assertNull(HiveTrackingController.findNearestTrackableCluster(
                Collections.singletonList(cluster), true));
        addCluster(38, 0, -100, -10, false);
        List<ClusterDetection> clusters = new BioBuzzLimelightClusterAdapter().detect(result, Alliance.BLUE);
        assertNull(HiveTrackingController.findNearestTrackableCluster(clusters, false));
        tags.clear();
        addCluster(38, Double.NaN, 100, -10, false);
        clusters = new BioBuzzLimelightClusterAdapter().detect(result, Alliance.BLUE);
        assertNull(HiveTrackingController.findNearestTrackableCluster(clusters, true));
    }

    @Test
    public void noScorableTargetReportsWarningAndClearsPreviousTarget() {
        addCluster(38, 40, 100, -10, false);
        assertTrue(tracking.update(true) > 0);
        tags.clear();
        addCluster(38, 40, 100, 10, false);

        assertEquals(0, tracking.update(true), 0);
        RobotTelemetry output = mock(RobotTelemetry.class);
        tracking.publishTelemetry(output);
        verify(output).status("Hive tracking", "NOT SCORABLE", RobotTelemetry.Level.ERROR);
        verify(output, never()).value(eq("Hive target"), any());
    }

    @Test
    public void stopClearsTargetAndVisionLifecycleStopsCamera() {
        addCluster(38, 40, 100, -10, false);
        tracking.update(true);
        tracking.stop();
        vision.stop();
        vision.stop();

        RobotTelemetry output = mock(RobotTelemetry.class);
        tracking.publishTelemetry(output);
        verify(output).status("Hive tracking", "MANUAL", RobotTelemetry.Level.INFO);
        verify(output).number("Tracking rotation request", 0, "power");
        verify(camera).pipelineSwitch(0);
        verify(camera).start();
        verify(camera).stop();
        assertThrows(IllegalStateException.class, () -> vision.update(Alliance.BLUE));
    }

    @Test
    public void turnCurveMatchesRearCameraTestDeadbandMinimumGainAndCap() {
        assertEquals(0, HiveTrackingController.getTurnPower(2), 0);
        assertEquals(0, HiveTrackingController.getTurnPower(-2), 0);
        assertEquals(0.12, HiveTrackingController.getTurnPower(3), 0);
        assertEquals(-0.40, HiveTrackingController.getTurnPower(-20), 1e-9);
        assertEquals(0.60, HiveTrackingController.getTurnPower(80), 0);
        assertEquals(-0.60, HiveTrackingController.getTurnPower(-80), 0);
        assertThrows(IllegalArgumentException.class,
                () -> HiveTrackingController.getTurnPower(Double.NaN));
    }

    @Test
    public void visionAgeIncludesProcessingLatencyButDoesNotDoubleCountParsing() {
        when(result.getStaleness()).thenReturn(50L);
        when(result.getCaptureLatency()).thenReturn(10.0);
        when(result.getTargetingLatency()).thenReturn(20.0);
        when(result.getParseLatency()).thenReturn(5.0);
        assertEquals(80, vision.update(Alliance.BLUE).observationAgeMs, 0);
        when(result.getStaleness()).thenReturn(220L);
        assertTrue(vision.update(Alliance.BLUE).hasValidResult);
        when(result.getStaleness()).thenReturn(221L);
        assertFalse(vision.update(Alliance.BLUE).hasValidResult);
        when(result.getCaptureLatency()).thenReturn(Double.NaN);
        assertFalse(vision.update(Alliance.BLUE).hasValidResult);
        when(result.getCaptureLatency()).thenReturn(-1.0);
        assertFalse(vision.update(Alliance.BLUE).hasValidResult);
        when(result.getCaptureLatency()).thenReturn(0.0);
        when(result.getTargetingLatency()).thenReturn(-1.0);
        assertFalse(vision.update(Alliance.BLUE).hasValidResult);
    }

    @Test
    public void motionCompensatedTrackingStillStopsImmediatelyWhenVisionIsLost() {
        long[] time = {0};
        ImuAimController aim = new ImuAimController(() -> -20, () -> time[0]);
        tracking = new HiveTrackingController(vision, preferences, aim);
        tracking.update(false);
        time[0] = 50_000_000;
        when(result.getStaleness()).thenReturn(50L);
        addCluster(38, 40, 100, -10, false);
        assertTrue(tracking.update(true) > 0);
        RobotTelemetry output = mock(RobotTelemetry.class);
        tracking.publishTelemetry(output);
        verify(output).status("Hive motion compensation", "IMU ROTATION", RobotTelemetry.Level.INFO);
        when(result.isValid()).thenReturn(false);
        time[0] = 60_000_000;
        assertEquals(0, tracking.update(true), 0);
        assertFalse(tracking.hasTarget());
    }

    private void addCluster(int firstId, double x, double z, double roll, boolean descending) {
        addTag(firstId, x + (descending ? 5 : -5), z, roll);
        addTag(firstId + 1, x + (descending ? -5 : 5), z, roll);
    }

    private void addTag(int id, double x, double z, double roll) {
        LLResultTypes.FiducialResult tag = mock(LLResultTypes.FiducialResult.class);
        when(tag.getFiducialId()).thenReturn(id);
        when(tag.getTargetPoseCameraSpace()).thenReturn(new Pose3D(
                new Position(DistanceUnit.INCH, x, 0, z, 0),
                new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, roll, 0)));
        tags.add(tag);
    }
}
