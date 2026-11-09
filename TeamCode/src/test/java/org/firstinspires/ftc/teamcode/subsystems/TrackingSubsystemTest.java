package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.*;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.BioBuzzLimelightClusterAdapter.Alliance;
import org.firstinspires.ftc.teamcode.TestProfiles;
import org.firstinspires.ftc.teamcode.subsystems.ConfigurationSubsystem.StartingSpot;
import org.junit.Before;
import org.junit.Test;

public class TrackingSubsystemTest {
    private TrackingSubsystem tracking;
    private SparkFunOTOS sensor;
    private SparkFunOTOS.Status status;
    private double x, y, h;
    private double vx, vy, vh;

    @Before public void setup() {
        HardwareMap hardware = mock(HardwareMap.class);
        sensor = mock(SparkFunOTOS.class);
        status = new SparkFunOTOS.Status();
        when(sensor.getStatus()).thenReturn(status);
        when(sensor.calibrateImu()).thenReturn(true);
        when(hardware.get(SparkFunOTOS.class, "otos")).thenReturn(sensor);
        doAnswer(call -> {
            SparkFunOTOS.Pose2D p = call.getArgument(0);
            p.x = x; p.y = y; p.h = h;
            SparkFunOTOS.Pose2D v = call.getArgument(1);
            v.x = vx; v.y = vy; v.h = vh;
            return null;
        }).when(sensor).getPosVelAcc(any(), any(), any());
        tracking = new TrackingSubsystem();
        tracking.init(hardware, TestProfiles.starter());
    }

    @Test public void allStartingPosesAndRawAxesTransformToField() {
        for (Alliance alliance : new Alliance[] {Alliance.RED, Alliance.BLUE}) {
            for (StartingSpot side : StartingSpot.values()) {
                x = y = h = 0;
                FieldMap.Pose start = FieldMap.startingPose(alliance, side, 18);
                tracking.start(start);
                assertEquals(alliance == Alliance.RED ? -12 : 12, tracking.getPose().x, 1e-9);
                assertEquals(side == StartingSpot.AUDIENCE ? -63 : 63, tracking.getPose().y, 1e-9);
                x = -10;
                y = 2;
                tracking.update(start.headingDegrees);
                double sign = side == StartingSpot.AUDIENCE ? 1 : -1;
                assertEquals(start.x + sign * 2, tracking.getPose().x, 1e-9);
                assertEquals(start.y + sign * 10, tracking.getPose().y, 1e-9);
            }
        }
    }

    @Test public void rotationCompensatesOpticalCenterOnce() {
        FieldMap.Pose start = new FieldMap.Pose(0, 0, 0);
        tracking.start(start);
        double forward = (19 - 45.72 / 2) / 2.54;
        double right = (45.72 / 2 - 7.5) / 2.54;
        // A stationary robot rotates 90 degrees: raw readings report sensor displacement.
        double dx = right - forward;
        double dy = forward + right;
        x = -dx;
        y = -dy;
        tracking.update(90);
        assertEquals(0, tracking.getPose().x, 1e-9);
        assertEquals(0, tracking.getPose().y, 1e-9);
        assertEquals(90, tracking.getPose().headingDegrees, 1e-9);
    }

    @Test public void invalidReadingOrSensorWarningLatchesLossUntilRestart() {
        tracking.start(new FieldMap.Pose(0, 0, 0));
        status.warnOpticalTracking = true;
        tracking.update(0);
        assertNull(tracking.getPose());
        status.warnOpticalTracking = false;
        tracking.update(0);
        assertNull(tracking.getPose());
        tracking.start(new FieldMap.Pose(0, 0, 0));
        assertNotNull(tracking.getPose());
        tracking.update(Double.NaN);
        assertNull(tracking.getPose());
    }

    @Test public void fieldLandmarksAndZonesMatchFieldLayout() {
        assertEquals(-9.4, FieldMap.hiveCell(Alliance.RED, StartingSpot.AUDIENCE).y, 0);
        assertEquals(9.4, FieldMap.hiveCell(Alliance.BLUE, StartingSpot.SCORING).y, 0);
        assertEquals(-72, FieldMap.garden(Alliance.RED).minX, 0);
        assertEquals(72, FieldMap.garden(Alliance.BLUE).maxY, 0);
        assertEquals(24, FieldMap.parking(Alliance.RED).minY, 0);
        assertEquals(-24, FieldMap.parking(Alliance.BLUE).maxY, 0);
        assertEquals(4, FieldMap.FLOWER_WALL_POINTS.size());
    }

    @Test public void invalidVelocityDoesNotChangeExistingPoseValidity() {
        tracking.start(new FieldMap.Pose(0, 0, 0));
        vx = Double.NaN;
        tracking.update(0);
        assertNotNull(tracking.getPose());
        assertTrue(tracking.hasValidPose());
        assertFalse(tracking.hasValidMotion());
        assertFalse(tracking.isStationary(1.5, 5));
        verify(sensor, times(1)).calibrateImu();
        assertFalse(tracking.calibrateImuIfStationary(1.5, 5));
        verify(sensor, times(1)).calibrateImu();
    }

    @Test public void motionTransformsToFieldAndRemovesRotatingSensorLeverArm() {
        tracking.start(new FieldMap.Pose(0, 0, 90));
        vx = -10;
        vy = 2;
        tracking.update(90, 0);
        assertEquals(2, tracking.getVelocityX(), 1e-9);
        assertEquals(10, tracking.getVelocityY(), 1e-9);
        assertFalse(tracking.isStationary(1.5, 5));
        vx = vy = 0;
        assertTrue(tracking.calibrateImuIfStationary(20, 5));

        tracking.start(new FieldMap.Pose(0, 0, 0));
        double forward = (19 - 45.72 / 2) / 2.54;
        double right = (45.72 / 2 - 7.5) / 2.54;
        double omega = Math.toRadians(30);
        vx = omega * forward;
        vy = -omega * right;
        tracking.update(90, 30);
        assertEquals(0, tracking.getSpeedInchesPerSecond(), 1e-9);
        assertEquals(30, tracking.getHeadingRateDegreesPerSecond(), 0);
        assertFalse(tracking.isStationary(1.5, 5));
    }
}
