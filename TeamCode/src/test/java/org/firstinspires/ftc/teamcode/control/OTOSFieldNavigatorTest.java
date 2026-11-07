package org.firstinspires.ftc.teamcode.control;

import static org.junit.Assert.*;

import org.firstinspires.ftc.teamcode.control.BioBuzzFieldMap.Point;
import org.firstinspires.ftc.teamcode.control.OTOSFieldNavigator.Command;
import org.junit.Test;

public class OTOSFieldNavigatorTest {
    private static final double SENSOR_BACK = 9 - 19 / 2.54;
    private static final double SENSOR_RIGHT = 7.5 / 2.54 - 9;
    private final OTOSFieldNavigator navigator = new OTOSFieldNavigator(SENSOR_BACK, SENSOR_RIGHT);

    @Test
    public void initializesKnownRobotCenterAndConvertsMeasuredAxes() {
        navigator.updatePose(0, 0, 0, 0, 0, 0);
        assertEquals(-63, navigator.getCenter().x, 1e-9);
        assertEquals(0, navigator.getCenter().y, 1e-9);
        navigator.updatePose(-12, 4, 0, -2, 1, 0);
        assertEquals(-51, navigator.getCenter().x, 1e-9);
        assertEquals(-4, navigator.getCenter().y, 1e-9);
    }

    @Test
    public void rotatingOffsetSensorDoesNotMoveRobotCenter() {
        for (double heading : new double[] {-179, -90, 0, 90, 179}) {
            setPose(-45, 0, heading, 0, 0, 45);
            assertEquals(-45, navigator.getCenter().x, 1e-9);
            assertEquals(0, navigator.getCenter().y, 1e-9);
            navigator.saveHome();
            navigator.start(0);
            // Compensated robot-center velocity is zero, but rotation is not stationary.
            assertTrue(navigator.isReturning());
            assertEquals(0, navigator.distanceToHome(), 1e-9);
        }
    }

    @Test
    public void savedHomePreservesGlobalLocalizationAndItsHeading() {
        setPose(-45, 10, 90, 0, 0, 0);
        navigator.saveHome();
        assertEquals("Home saved; field coordinates preserved", navigator.getStatus());
        setPose(-45, 10, 0, 0, 0, 0);
        navigator.start(0);
        assertTrue(navigator.update(0).clockwise < 0);
        assertEquals(-45, navigator.getCenter().x, 1e-9);
        assertEquals(10, navigator.getCenter().y, 1e-9);
    }

    @Test
    public void refusesUnknownOrUnsafeEndpointsInsteadOfDrivingStraightThroughFrame() {
        setPose(0, 0, 0, 0, 0, 0);
        navigator.start(0);
        assertFalse(navigator.isReturning());
        assertTrue(navigator.getStatus().startsWith("Return refused"));
        navigator.saveHome();
        assertTrue(navigator.getStatus().startsWith("Home refused"));
        setPose(-63, 20, 0, 0, 0, 0);
        navigator.start(0);
        assertFalse(navigator.isReturning());
        navigator.updatePose(Double.NaN, 0, 0, 0, 0, 0);
        navigator.start(0);
        assertFalse(navigator.isReturning());
    }

    @Test
    public void returnUsesWaypointsFromBlueSideAndStopsOnRouteDeviation() {
        setPose(45, 0, 0, 0, 0, 0);
        navigator.start(0);
        assertEquals(3, navigator.remainingWaypoints());
        navigator.update(0);
        setPose(0, 0, 0, 0, 0, 0);
        assertSame(Command.STOP, navigator.update(0.1));
        assertFalse(navigator.isReturning());
        assertTrue(navigator.getStatus().contains("outside the clear route"));
    }

    @Test
    public void stallsTimeoutAndCancellationStopMotion() {
        setPose(-45, 0, 0, 0, 0, 0);
        navigator.start(0);
        navigator.update(0);
        assertSame(Command.STOP, navigator.update(4));
        assertTrue(navigator.getStatus().contains("no progress"));
        navigator.start(5);
        assertSame(Command.STOP, navigator.update(65));
        assertTrue(navigator.getStatus().contains("timed out"));
        navigator.start(70);
        navigator.cancel("Manual override");
        assertSame(Command.STOP, navigator.update(71));
    }

    @Test
    public void dockingDoesNotTurnOrStrafeAndStopsShortOfWall() {
        setPose(-55, 0, 0, 0, 0, 0);
        navigator.start(0);
        Command command = navigator.update(0);
        assertTrue(command.forward < 0);
        assertEquals(0, command.clockwise, 0);
        assertEquals(0, command.right, 0);
        assertTrue(Math.abs(command.forward) <= 0.25);
        setPose(-62, 0, 0, 0, 0, 0);
        assertSame(Command.STOP, navigator.update(1));
        navigator.update(1.21);
        assertFalse(navigator.isReturning());
        assertEquals("Home reached", navigator.getStatus());
    }

    @Test
    public void dockingStopsIfHeadingOrLateralAlignmentIsLost() {
        setPose(-55, 0, 0, 0, 0, 0);
        navigator.start(0);
        setPose(-56, 1, 0, 0, 0, 0);
        assertSame(Command.STOP, navigator.update(0.2));
        assertTrue(navigator.getStatus().contains("misaligned"));
    }

    @Test
    public void refusesToContinueWhenMeasuredCoastingMotionWouldEnterFrameClearance() {
        setPose(-42, 0, 0, 12, 0, 0);
        navigator.start(0);
        assertSame(Command.STOP, navigator.update(0));
        assertTrue(navigator.getStatus().contains("insufficient clearance"));
    }

    @Test
    public void invalidHomeDoesNotOverwritePreviouslySavedHome() {
        setPose(-45, 10, 0, 0, 0, 0);
        navigator.saveHome();
        setPose(0, 0, 0, 0, 0, 0);
        navigator.saveHome();
        assertTrue(navigator.getStatus().startsWith("Home refused"));
        setPose(-45, 10, 0, 0, 0, 0);
        navigator.start(0);
        assertEquals("Home reached", navigator.getStatus());
    }

    @Test
    public void savedHomeAcrossFrameIsReachedWithSavedHeadingAndGlobalPoseIntact() {
        setPose(-45, 20, 45, 0, 0, 0);
        navigator.saveHome();
        simulateReturn(45, -25, -110, new Point(-45, 20), 45, false);
    }

    @Test
    public void simulatedReturnFromAcrossFrameMaintainsClearanceAndDocks() {
        simulateReturn(45, 0, -100);
    }

    @Test
    public void simulatedReturnFromOppositeCornerMaintainsClearanceAndDocks() {
        simulateReturn(45, 45, 90);
    }

    private void simulateReturn(double x, double y, double heading) {
        simulateReturn(x, y, heading, new Point(-62, 0), 0, true);
    }

    private void simulateReturn(double x, double y, double heading,
            Point goal, double goalHeading, boolean docking) {
        BioBuzzFieldMap map = new BioBuzzFieldMap();
        double vx = 0;
        double vy = 0;
        double turnRate = 0;
        setPose(x, y, heading, vx, vy, turnRate);
        navigator.start(0);
        assertTrue(navigator.isReturning());
        for (int step = 0; step < 3000 && navigator.isReturning(); step++) {
            setPose(x, y, heading, vx, vy, turnRate);
            Command command = navigator.update(step * 0.02);
            // Match DriveSubsystem's prioritized mecanum allocation at the default 45% power.
            double translation = Math.abs(command.forward) + Math.abs(command.right);
            double scale = translation > 1 - Math.abs(command.clockwise)
                    ? (1 - Math.abs(command.clockwise)) / translation : 1;
            double forward = command.forward * scale * 0.45 * 30;
            double right = command.right * scale * 0.45 * 30;
            turnRate = -command.clockwise * 0.45 * 120;
            double h = Math.toRadians(heading);
            vx = forward * Math.cos(h) + right * Math.sin(h);
            vy = forward * Math.sin(h) - right * Math.cos(h);
            x += vx * 0.02;
            y += vy * 0.02;
            heading += turnRate * 0.02;
            if (x >= -BioBuzzFieldMap.CENTER_LIMIT) {
                assertTrue("Unsafe position " + x + ", " + y, map.isClear(new Point(x, y)));
            } else {
                assertTrue(docking);
                assertTrue(Math.abs(y) <= 0.75);
                assertTrue(Math.abs(heading) <= 3);
                assertTrue(x >= -63);
            }
        }
        assertEquals("Home reached", navigator.getStatus());
        assertEquals(goal.x, x, 0.51);
        assertEquals(goal.y, y, docking ? 0.75 : 0.51);
        assertEquals(goalHeading, heading, 3);
    }

    private void setPose(double x, double y, double heading, double vx, double vy, double turnRate) {
        double h = Math.toRadians(heading);
        double ox = Math.cos(h) * SENSOR_BACK - Math.sin(h) * SENSOR_RIGHT;
        double oy = Math.sin(h) * SENSOR_BACK + Math.cos(h) * SENSOR_RIGHT;
        double omega = Math.toRadians(turnRate);
        navigator.updatePose(OTOSFieldNavigator.START_X - x + ox - SENSOR_BACK,
                -y + oy - SENSOR_RIGHT, heading,
                -vx - omega * oy, -vy + omega * ox, turnRate);
    }
}
