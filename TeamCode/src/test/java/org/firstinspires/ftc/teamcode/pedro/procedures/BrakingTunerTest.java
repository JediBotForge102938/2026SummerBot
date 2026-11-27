package org.firstinspires.ftc.teamcode.pedro.procedures;

import static org.junit.Assert.*;

import com.pedropathing.algorithm.ForesightConfig;
import java.util.Arrays;
import java.util.List;
import org.firstinspires.ftc.teamcode.pedro.PedroConstants;
import org.junit.Test;

public class BrakingTunerTest {
    @Test
    public void selectsOnlyRequestedTestAndHeadingDoesNotRequireExistingTuning() {
        BrakingTuner tuner = new BrakingTuner(h -> null, h -> null,
                PedroConstants::competitionForesightConfig);
        ForesightConfig config = PedroConstants.competitionForesightConfig();
        ForwardBraking forward = (ForwardBraking) tuner.createStep(
                BrakingTuner.Test.FORWARD, 36, config, h -> null);
        StrafeBraking strafe = (StrafeBraking) tuner.createStep(
                BrakingTuner.Test.STRAFE, 24, config, h -> null);
        assertEquals(36, forward.distance, 0);
        assertEquals(24, strafe.distance, 0);
        assertTrue(tuner.createStep(BrakingTuner.Test.HEADING, 0, null, h -> null)
                instanceof HeadingBraking);
    }

    @Test
    public void generatedCodePreservesUntestedAxis() {
        ForesightConfig config = PedroConstants.competitionForesightConfig();
        String forward = BrakingTuner.replacementCode(BrakingTuner.Test.FORWARD, List.of(0.2, 0.003), config);
        assertTrue(forward.contains("Matrix.diag(0.2, " + config.linearBrakeCoefficients.get().get(1, 1)));
        assertTrue(forward.contains("Matrix.diag(0.003, " + config.quadraticBrakeCoefficients.get().get(1, 1)));
        String strafe = BrakingTuner.replacementCode(BrakingTuner.Test.STRAFE, List.of(0.4, 0.005), config);
        assertTrue(strafe.contains("Matrix.diag(" + config.linearBrakeCoefficients.get().get(0, 0) + ", 0.4)"));
        assertTrue(strafe.contains("Matrix.diag(" + config.quadraticBrakeCoefficients.get().get(0, 0) + ", 0.005)"));
        assertEquals("c.headingBrakeCoefficients.set(Vector2D.cartesian(0.2, 0.003));",
                BrakingTuner.replacementCode(BrakingTuner.Test.HEADING, List.of(0.2, 0.003), null));
    }

    @Test
    public void rejectsIncompleteMeasurementsAndInvalidDistances() {
        assertFalse(BrakingTuner.validCoefficients(null));
        assertFalse(BrakingTuner.validCoefficients(List.of()));
        assertFalse(BrakingTuner.validCoefficients(List.of(1.0, Double.NaN)));
        assertFalse(BrakingTuner.validCoefficients(Arrays.asList(null, 1.0)));
        assertTrue(BrakingTuner.validCoefficients(List.of(0.1, 0.002)));
        assertFalse(BrakingTuner.validDistance(null));
        assertFalse(BrakingTuner.validDistance(Double.POSITIVE_INFINITY));
        assertFalse(BrakingTuner.validDistance(14.0));
        assertTrue(BrakingTuner.validDistance(15.0));
    }
}
