package org.firstinspires.ftc.teamcode.control;

import static org.junit.Assert.*;

import org.junit.Test;

public class LandmarkTravelControlsTest {
    @Test public void startDoesNotLaunchHeldAAndFreshAStartsOrCancels() {
        LandmarkTravelControls controls = new LandmarkTravelControls();
        LandmarkTravelControls.Input input = new LandmarkTravelControls.Input();
        input.a = true;
        controls.seed(input);
        assertFalse(controls.update(input, false).toggleRoute);
        input.a = false;
        controls.update(input, false);
        input.a = true;
        assertTrue(controls.update(input, false).toggleRoute);
        input.a = false;
        controls.update(input, true);
        input.a = true;
        assertTrue(controls.update(input, true).toggleRoute);
        assertFalse(controls.update(input, true).toggleRoute);
    }

    @Test public void stopButtonsOverrideOtherInputAndManualCancelsInSameLoop() {
        LandmarkTravelControls controls = new LandmarkTravelControls();
        LandmarkTravelControls.Input input = new LandmarkTravelControls.Input();
        controls.seed(input);
        input.a = true;
        input.leftY = -0.5;
        LandmarkTravelControls.Actions manual = controls.update(input, true);
        assertTrue(manual.manual);
        assertTrue(manual.cancel);
        assertFalse(manual.toggleRoute);

        input.a = false;
        input.y = true;
        input.x = true;
        input.leftY = 0;
        LandmarkTravelControls.Actions stopped = controls.update(input, true);
        assertTrue(stopped.stop);
        assertTrue(stopped.cancel);
        assertFalse(stopped.calibrate);
        assertFalse(stopped.manual);
    }

    @Test public void dpadEdgesSelectAndSaveCalibrationEdgesAreRisingOnly() {
        LandmarkTravelControls controls = new LandmarkTravelControls();
        LandmarkTravelControls.Input input = new LandmarkTravelControls.Input();
        controls.seed(input);
        input.dpadDown = true;
        assertEquals(1, controls.update(input, false).selectionDelta);
        assertEquals(0, controls.update(input, false).selectionDelta);
        input.dpadDown = false;
        controls.update(input, false);
        input.options = true;
        assertTrue(controls.update(input, false).saveHome);
        assertFalse(controls.update(input, false).saveHome);
        input.options = false;
        controls.update(input, false);
        input.x = true;
        assertTrue(controls.update(input, false).calibrate);
        assertFalse(controls.update(input, false).calibrate);
    }
}
