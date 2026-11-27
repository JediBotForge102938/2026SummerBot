package org.firstinspires.ftc.teamcode.pedro.procedures;

import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.OTOSConfig;
import com.pedropathing.revhub.localizers.OTOSLocalizer;
import com.pedropathing.tuning.autotune.Inputs;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.TuningOpMode;
import com.pedropathing.utils.Angle;

import org.firstinspires.ftc.teamcode.pedro.PedroConstants;
import org.firstinspires.ftc.teamcode.subsystems.RobotProfile;
import java.util.List;
import java.util.Locale;

public class OTOSTuner extends Procedure {
    public OTOSTuner() {
        super("OTOS Tuner", "A procedure for tuning the OTOS localizer.");
    }

    @Override
    public void run() throws InterruptedException {
        RobotProfile profile = RobotProfile.loadActive(null);
        double mountingRotation = profile.otosMountingRotationDegrees;
        Inputs inputs = inputs("Setup", "Set OTOS HardwareMap Name");
        Inputs.Field<String> name = inputs.s("HardwareMap Name").withDefault(profile.otosName);
        awaitInputs(inputs);

        Inputs scalar = inputs(
                "Scalar Identification",
                "Set the distance you will push your robot forward in inches and the number of full rotations for the angular test"
        );
        Inputs.Field<Double> distance = scalar.d("Distance to push robot").withDefault(48.0);
        Inputs.Field<Integer> turns = scalar.i("Full rotations").withDefault(10);
        awaitInputs(scalar);

        if (distance.get() == null || !Double.isFinite(distance.get()) || distance.get() <= 0.0) {
            abort("Enter a positive push distance in inches.");
            return;
        }
        if (turns.get() <= 0) {
            abort("Enter a positive number of full rotations.");
            return;
        }

        Double angularScalar = runOpMode(new OTOSAngularScalar(name.get(), turns.get(), mountingRotation));
        if (!validScalar(angularScalar)) {
            abort("Invalid angular scalar " + angularScalar + ". Check OTOS tracking and repeat the angular test.");
            return;
        }
        Double linearScalar = runOpMode(new OTOSLinearScalar(name.get(), distance.get(), mountingRotation));
        if (!validScalar(linearScalar)) {
            abort("Invalid linear scalar " + linearScalar
                    + ". Check the OTOS mounting and forward X tracking before repeating the test.");
            return;
        }

        List<Double> offsets = runOpMode(new OTOSOffsets(name.get(), linearScalar, angularScalar, mountingRotation));
        if (offsets == null) {
            abort("Offset stage ended without a saved pose. Rotate the robot 180 degrees about the robot center, then press Stop.");
            return;
        }

        result("name", name.get());
        result("linearScalar", linearScalar);
        result("angularScalar", angularScalar);
        result("xOffset", offsets.get(0));
        result("yOffset", offsets.get(1));

        code(Language.JAVA,"public static OTOSConfig localizerConfig = new OTOSConfig(c -> {\n" +
                "    c.name.set(\"" + name.get() + "\");\n" +
                "    c.linearScalar.set(" + linearScalar + ");\n" +
                "    c.angularScalar.set(" + angularScalar + ");\n" +
                "    c.offset.set(new Pose(" + offsets.get(0) + ", " + offsets.get(1) +
                ", Math.toRadians(" + mountingRotation + ")));\n" +
                "    c.linearUnit.set(DistanceUnit.INCH);\n" +
                "});");
    }

    static boolean validScalar(Double value) {
        return value != null && Double.isFinite(value) && value >= 0.872 && value <= 1.127;
    }
}

class OTOSLinearScalar extends TuningOpMode<Double> {
    String name;
    double distance;
    double mountingRotation;

    public OTOSLinearScalar(String name, double distance, double mountingRotation) {
        super("Linear Scalar Identification",
                "Determines the linear scalar for the OTOS localizer. \n"
                        + "Push your robot forward " + distance + " inches, stop moving, then press Stop",
                true);
        this.name = name;
        this.distance = distance;
        this.mountingRotation = mountingRotation;
    }

    @Override
    protected Double runTuningOpMode() {
        OTOSConfig config = PedroConstants.otosConfig(name, mountingRotation, 1.0, 1.0);
        OTOSLocalizer localizer = new OTOSLocalizer(hardwareMap, config);
        waitForStart();
        if (isStopRequested()) {
            throw new IllegalStateException("Linear scalar step stopped before it could start sampling.");
        }
        localizer.setPose(Pose.zero());
        localizer.update();

        Pose position = null;

        while (!isStopRequested()) {
            localizer.update();
            position = localizer.pose();
            telemetry.addData("OTOS position (in)", "X %.2f, Y %.2f", position.x(), position.y());
            telemetry.addData("OTOS heading (rad)", "%.2f", position.heading());
            telemetry.addData("Push distance (in)", "%.2f", distance);
            telemetry.update();
        }

        if (position == null) {
            throw new IllegalStateException("Linear scalar step stopped before any position was sampled.");
        }
        if (!Double.isFinite(position.x()) || Math.abs(position.x()) <= 1e-9) {
            throw new IllegalStateException(String.format(Locale.US,
                    "Linear scalar step measured X = %.2f in and Y = %.2f in. Check live OTOS position before stopping.",
                    position.x(), position.y()));
        }

        return Math.abs(distance / position.x());
    }
}

class OTOSAngularScalar extends TuningOpMode<Double> {
    String name;
    int turns;
    double targetRadians;
    double mountingRotation;

    public OTOSAngularScalar(String name, int turns, double mountingRotation) {
        super("Angular Scalar Identification",
                "Determines the angular scalar for the OTOS localizer. \n"
                        + "Spin your robot " + turns + " full rotations, stop moving, then press Stop",
                true);
        this.name = name;
        this.turns = turns;
        this.targetRadians = turns * 2.0 * Math.PI;
        this.mountingRotation = mountingRotation;
    }

    @Override
    protected Double runTuningOpMode() {
        OTOSConfig config = PedroConstants.otosConfig(name, mountingRotation, 1.0, 1.0);
        OTOSLocalizer localizer = new OTOSLocalizer(hardwareMap, config);
        localizer.setPose(Pose.zero());
        localizer.update();

        waitForStart();

        localizer.update();
        double prevHeading = localizer.pose().heading();
        double totalHeading = 0.0;

        while (!isStopRequested()) {
            localizer.update();

            Pose position = localizer.pose();
            double currentHeading = position.heading();
            totalHeading += Angle.normalizeSigned(currentHeading - prevHeading);
            prevHeading = currentHeading;


        }

        if (Math.abs(totalHeading) <= 1e-9) {
            return null;
        }

        return Math.abs(targetRadians / totalHeading);
    }
}

class OTOSOffsets extends TuningOpMode<List<Double>> {
    String name;
    double linearScalar, angularScalar;
    double mountingRotation;

    public OTOSOffsets(String name, double linearScalar, double angularScalar, double mountingRotation) {
        super("OTOS Offset Identification",
                "Automatically identifies the X/Y offset for your OTOS localizer. \n"
                        + "Rotate the robot 180 degrees counterclockwise about the robot center without translating it, stop moving, then press Stop",
                true);
        this.name = name;
        this.linearScalar = linearScalar;
        this.angularScalar = angularScalar;
        this.mountingRotation = mountingRotation;
    }

    @Override
    protected List<Double> runTuningOpMode() {
        OTOSConfig config = PedroConstants.otosConfig(name, mountingRotation, linearScalar, angularScalar);
        OTOSLocalizer localizer = new OTOSLocalizer(hardwareMap, config);
        localizer.setPose(Pose.zero());
        localizer.update();

        Pose position = null;

        waitForStart();

        while (!isStopRequested()) {
            localizer.update();
            position = localizer.pose();
            telemetry.addData("heading", localizer.pose().heading());
            telemetry.update();
        }

        if (position == null) {
            return null;
        }

        return List.of(-position.x() / 2.0, -position.y() / 2.0);
    }
}
