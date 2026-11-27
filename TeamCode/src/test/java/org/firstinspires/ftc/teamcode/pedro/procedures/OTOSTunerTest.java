package org.firstinspires.ftc.teamcode.pedro.procedures;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.OTOSLocalizer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.Test;
import org.mockito.MockedConstruction;

public class OTOSTunerTest {
    @Test
    public void rejectsUnusableScalarResults() {
        assertTrue(OTOSTuner.validScalar(0.998));
        assertTrue(OTOSTuner.validScalar(0.872));
        assertTrue(OTOSTuner.validScalar(1.127));
        assertFalse(OTOSTuner.validScalar(55.487));
        assertFalse(OTOSTuner.validScalar(Double.NaN));
        assertFalse(OTOSTuner.validScalar(null));
    }

    @Test
    public void linearScalarUsesCapturedForwardMovement() {
        try (MockedConstruction<OTOSLocalizer> ignored = mockConstruction(OTOSLocalizer.class,
                (localizer, context) -> when(localizer.pose()).thenReturn(new Pose(36, 2, 0)))) {
            OTOSLinearScalar tuner = tuner();
            doReturn(false, false, true).when(tuner).isStopRequested();
            assertEquals(48.0 / 36.0, tuner.runTuningOpMode(), 1e-9);
            verify(tuner.telemetry).addData(eq("OTOS position (in)"), anyString(), any(), any());
        }
    }

    @Test
    public void linearScalarDistinguishesNoSampleFromZeroForwardMotion() {
        try (MockedConstruction<OTOSLocalizer> ignored = mockConstruction(OTOSLocalizer.class,
                (localizer, context) -> when(localizer.pose()).thenReturn(new Pose(0, 5, 0)))) {
            OTOSLinearScalar noSample = tuner();
            doReturn(false, true).when(noSample).isStopRequested();
            IllegalStateException noSampleError =
                    assertThrows(IllegalStateException.class, noSample::runTuningOpMode);
            assertTrue(noSampleError.getMessage().contains("before any position was sampled"));

            OTOSLinearScalar noForward = tuner();
            doReturn(false, false, true).when(noForward).isStopRequested();
            IllegalStateException noForwardError =
                    assertThrows(IllegalStateException.class, noForward::runTuningOpMode);
            assertTrue(noForwardError.getMessage().contains("X = 0.00 in and Y = 5.00 in"));
        }
    }

    private static OTOSLinearScalar tuner() {
        OTOSLinearScalar tuner = spy(new OTOSLinearScalar("otos", 48, -90));
        tuner.hardwareMap = mock(HardwareMap.class);
        tuner.telemetry = mock(Telemetry.class);
        doNothing().when(tuner).waitForStart();
        return tuner;
    }
}
