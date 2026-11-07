package org.firstinspires.ftc.teamcode.control;

import static org.junit.Assert.*;

import org.junit.Test;

public class ImuAimControllerTest {
    private long time;
    private double rate;
    private final ImuAimController aim = new ImuAimController(() -> rate, () -> time);

    @Test
    public void clockwiseMotionReducesRearCameraErrorAndTurnPower() {
        sample(0, -100);
        sample(100, -100);
        assertEquals(0.20, aim.turn(30, 100), 1e-9);
        assertEquals(20, aim.getCorrectedAngle(), 1e-9);
        assertTrue(aim.isCompensated());
    }

    @Test
    public void counterclockwiseMotionReducesNegativeErrorAndBrakesInOppositeDirection() {
        sample(0, 100);
        sample(100, 100);
        assertEquals(-0.20, aim.turn(-30, 100), 1e-9);
        assertEquals(-20, aim.getCorrectedAngle(), 1e-9);
    }

    @Test
    public void integratesChangingRatesAndInterpolatesInsideSampleInterval() {
        sample(0, 0);
        sample(100, 100);
        aim.turn(10, 50);
        assertEquals(13.75, aim.getCorrectedAngle(), 1e-9);
        assertTrue(aim.isCompensated());
    }

    @Test
    public void repeatedImageUsesOnlyRotationSinceItsCapture() {
        sample(0, -100);
        sample(50, -100);
        aim.turn(30, 50);
        assertEquals(25, aim.getCorrectedAngle(), 1e-9);
        sample(100, -100);
        aim.turn(30, 100);
        assertEquals(20, aim.getCorrectedAngle(), 1e-9);
        aim.turn(15, 25);
        assertEquals(12.5, aim.getCorrectedAngle(), 1e-9);
    }

    @Test
    public void noHistoryExtrapolationDuringWarmupOrAfterLoopStall() {
        sample(0, -100);
        aim.turn(30, 50);
        assertFalse(aim.isCompensated());
        assertEquals(30, aim.getCorrectedAngle(), 0);
        sample(50, -100);
        aim.turn(30, 50);
        assertTrue(aim.isCompensated());
        sample(151, -100);
        aim.turn(30, 50);
        assertFalse(aim.isCompensated());
        assertEquals(30, aim.getCorrectedAngle(), 0);
    }

    @Test
    public void longRunningHistoryIsBoundedAndOldImagesAreNotExtrapolated() {
        for (int ms = 0; ms <= 2000; ms += 50) {
            sample(ms, 20);
        }
        aim.turn(0, 250);
        assertTrue(aim.isCompensated());
        assertEquals(5, aim.getCorrectedAngle(), 1e-9);
        aim.turn(0, 501);
        assertFalse(aim.isCompensated());
        assertEquals(0, aim.getCorrectedAngle(), 0);
    }

    @Test
    public void dampingBrakesAtTargetButSettlesToZeroAtLowRate() {
        sample(0, -50);
        assertEquals(-0.10, aim.turn(0, 0), 1e-9);
        sample(50, 5);
        assertEquals(0, aim.turn(2, 0), 0);
        sample(100, -5);
        assertEquals(0, aim.turn(-2, 0), 0);
    }

    @Test
    public void stationaryImuPreservesOriginalTurnCurveAndOutputIsCapped() {
        sample(0, 0);
        for (double angle : new double[] {-80, -20, -3, -2, 0, 2, 3, 20, 80}) {
            assertEquals(HiveTrackingController.getTurnPower(angle), aim.turn(angle, 0), 0);
        }
        sample(50, 100);
        assertEquals(0.60, aim.turn(80, 0), 0);
        sample(100, -100);
        assertEquals(-0.60, aim.turn(-80, 0), 0);
    }

    @Test
    public void stopClearsHistoryAndInvalidSensorDataFailsExplicitly() {
        sample(0, 0);
        aim.reset();
        assertThrows(IllegalStateException.class, () -> aim.turn(10, 0));
        rate = Double.NaN;
        assertThrows(IllegalStateException.class, aim::sample);
        sample(50, 0);
        assertThrows(IllegalArgumentException.class, () -> aim.turn(Double.NaN, 0));
        assertThrows(IllegalArgumentException.class, () -> aim.turn(10, Double.NaN));
        assertThrows(IllegalArgumentException.class, () -> aim.turn(10, -1));
    }

    private void sample(long ms, double degreesPerSecond) {
        time = ms * 1_000_000;
        rate = degreesPerSecond;
        aim.sample();
    }
}
