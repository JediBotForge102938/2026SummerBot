package org.firstinspires.ftc.teamcode.pedro.procedures;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector2D;
import com.pedropathing.tuning.autotune.DisplayName;
import com.pedropathing.tuning.autotune.Inputs;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.TuningOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.List;
import java.util.function.Function;
import java.util.function.Supplier;

public final class BrakingTuner extends Procedure {
    enum Test {
        @DisplayName("Forward Braking") FORWARD,
        @DisplayName("Strafe Braking") STRAFE,
        @DisplayName("Heading Braking") HEADING
    }

    private final Function<HardwareMap, Localizer> localizerFunction;
    private final Function<HardwareMap, Drivetrain> drivetrainFunction;
    private final Supplier<ForesightConfig> configSupplier;

    public BrakingTuner(Function<HardwareMap, Localizer> localizerFunction,
            Function<HardwareMap, Drivetrain> drivetrainFunction,
            Supplier<ForesightConfig> configSupplier) {
        super("Braking Only", "Repeat one braking test without running the full Foresight tuner. "
                + "These tests move automatically and may overshoot. Verify stable OTOS tracking first.");
        this.localizerFunction = localizerFunction;
        this.drivetrainFunction = drivetrainFunction;
        this.configSupplier = configSupplier;
    }

    @Override
    public void run() throws InterruptedException {
        Inputs selection = inputs("Braking test", "Choose one test. Distance is in inches and only "
                + "applies to forward/strafe tests. Keep extra space beyond the test's endpoints.");
        Inputs.Field<Test> selected = selection.e("Test", Test.class).withDefault(Test.FORWARD);
        Inputs.Field<Double> distance = selection.d("Distance").withDefault(36.0);
        awaitInputs(selection);
        if (selected.get() == null) {
            abort("Select a braking test.");
            return;
        }
        if (selected.get() != Test.HEADING && !validDistance(distance.get())) {
            abort("Enter a finite distance of at least 15 inches.");
            return;
        }

        Drivetrain[] activeDrive = new Drivetrain[1];
        Function<HardwareMap, Drivetrain> drive = hardware -> {
            activeDrive[0] = drivetrainFunction.apply(hardware);
            return activeDrive[0];
        };
        List<Double> coefficients;
        ForesightConfig config;
        try {
            config = selected.get() == Test.HEADING ? null : configSupplier.get();
            coefficients = runOpMode(createStep(selected.get(),
                    selected.get() == Test.HEADING ? 0 : distance.get(), config, drive));
        } catch (RuntimeException exception) {
            abort(selected.get() + " braking failed: " + exception.getMessage());
            throw exception;
        } finally {
            if (activeDrive[0] != null) activeDrive[0].stop();
        }
        if (!validCoefficients(coefficients)) {
            abort("Braking test ended without two finite coefficients. Check OTOS tracking before retrying.");
            return;
        }
        String prefix = selected.get() == Test.FORWARD ? "forward"
                : selected.get() == Test.STRAFE ? "strafe" : "heading";
        result(prefix + "BrakingLinearCoefficient", coefficients.get(0));
        result(prefix + "BrakingQuadraticCoefficient", coefficients.get(1));
        code(Language.JAVA, replacementCode(selected.get(), coefficients, config));
        if (coefficients.get(0) < 0 || coefficients.get(1) < 0) {
            result("Warning", "The fit contains a negative coefficient. Do not apply it without "
                    + "checking its braking predictions; negative quadratic fits can produce NaN path targets.");
        }
    }

    TuningOpMode<List<Double>> createStep(Test test, double distance, ForesightConfig config,
            Function<HardwareMap, Drivetrain> drive) {
        if (test == Test.HEADING) return new HeadingBraking(localizerFunction, drive);
        Vector2D heading = config.headingBrakeCoefficients.get();
        // These existing tests accept a proportional heading gain, as used by our constants.
        double headingKP = config.headingFeedback.get().calculate(0, 1);
        if (!Double.isFinite(headingKP) || headingKP <= 0
                || !Double.isFinite(heading.x()) || !Double.isFinite(heading.y())) {
            throw new IllegalStateException("Current heading tuning is not finite or has a non-positive gain");
        }
        return test == Test.FORWARD
                ? new ForwardBraking(localizerFunction, drive, heading.x(), heading.y(), headingKP, distance)
                : new StrafeBraking(localizerFunction, drive, heading.x(), heading.y(), headingKP, distance);
    }

    static boolean validDistance(Double distance) {
        return distance != null && Double.isFinite(distance) && distance >= 15;
    }

    static boolean validCoefficients(List<Double> coefficients) {
        return coefficients != null && coefficients.size() == 2
                && coefficients.get(0) != null && Double.isFinite(coefficients.get(0))
                && coefficients.get(1) != null && Double.isFinite(coefficients.get(1));
    }

    static String replacementCode(Test test, List<Double> coefficients, ForesightConfig config) {
        if (test == Test.HEADING) {
            return "c.headingBrakeCoefficients.set(Vector2D.cartesian("
                    + coefficients.get(0) + ", " + coefficients.get(1) + "));";
        }
        double forwardLinear = test == Test.FORWARD ? coefficients.get(0)
                : config.linearBrakeCoefficients.get().get(0, 0);
        double strafeLinear = test == Test.STRAFE ? coefficients.get(0)
                : config.linearBrakeCoefficients.get().get(1, 1);
        double forwardQuadratic = test == Test.FORWARD ? coefficients.get(1)
                : config.quadraticBrakeCoefficients.get().get(0, 0);
        double strafeQuadratic = test == Test.STRAFE ? coefficients.get(1)
                : config.quadraticBrakeCoefficients.get().get(1, 1);
        return "c.linearBrakeCoefficients.set(Matrix.diag(" + forwardLinear + ", " + strafeLinear + "));\n"
                + "c.quadraticBrakeCoefficients.set(Matrix.diag(" + forwardQuadratic + ", " + strafeQuadratic + "));";
    }
}
