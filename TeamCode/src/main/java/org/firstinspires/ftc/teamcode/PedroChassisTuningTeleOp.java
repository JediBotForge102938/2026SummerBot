package org.firstinspires.ftc.teamcode;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.control.ProfiledPedroFollower;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FieldMap;
import org.firstinspires.ftc.teamcode.subsystems.RobotProfile;
import org.firstinspires.ftc.teamcode.subsystems.RobotProfileTuningStore;
import org.firstinspires.ftc.teamcode.subsystems.TrackingSubsystem;

import java.util.Locale;
import java.io.IOException;

@TeleOp(name = "Pedro Chassis Tuning", group = "2 - (Tests)")
public final class PedroChassisTuningTeleOp extends OpMode {
    private enum Mode {
        LOCALIZATION, POWER_FORWARD, POWER_LEFT, POWER_TURN,
        MEASURE_FORWARD, MEASURE_LEFT, HEADING, POSITION_FORWARD, POSITION_LEFT,
        PATH_FORWARD, PATH_LEFT
    }

    private enum Parameter {
        POWER("testPower", 0.05, 0.05, 1),
        DISTANCE("testDistanceIn", 1, 1, 24),
        ANGLE("testAngleDegrees", 5, 5, 90),
        TRANSLATIONAL_P("translationalP", 0.01, 0, 10),
        TRANSLATIONAL_D("translationalD", 0.001, 0, 10),
        TRANSLATIONAL_F("translationalF", 0.005, 0, 1),
        HEADING_P("headingP", 0.05, 0, 10),
        HEADING_D("headingD", 0.001, 0, 10),
        HEADING_F("headingF", 0.005, 0, 1),
        DRIVE_P("driveP", 0.005, 0, 10),
        DRIVE_D("driveD", 0.001, 0, 10),
        DRIVE_F("driveF", 0.005, 0, 1),
        FORWARD_SPEED("forwardVelocity", 1, 1, 200),
        LATERAL_SPEED("lateralVelocity", 1, 1, 200),
        MASS("massKg", 0.1, 0.1, 100),
        FORWARD_ACCEL("forwardZeroPowerAcceleration", 1, -1000, -0.1),
        LATERAL_ACCEL("lateralZeroPowerAcceleration", 1, -1000, -0.1),
        MEASUREMENT_SECONDS("measurementPoweredSeconds", 0.1, 0.5, 2);

        final String key;
        final double step;
        final double min;
        final double max;

        Parameter(String key, double step, double min, double max) {
            this.key = key;
            this.step = step;
            this.min = min;
            this.max = max;
        }
    }

    private final DriveSubsystem drive = new DriveSubsystem();
    private final TrackingSubsystem tracking = new TrackingSubsystem();
    private final double[] values = new double[Parameter.values().length];
    private RobotProfile profile;
    private Follower follower;
    private RobotProfileTuningStore tuningStore;
    private String persistenceStatus = "Loaded controller profile";
    private Mode mode = Mode.LOCALIZATION;
    private Parameter parameter = Parameter.POWER;
    private boolean driveReady;
    private boolean started;
    private boolean active;
    private boolean coasting;
    private int direction = 1;
    private boolean previousA, previousX, previousUp, previousDown;
    private boolean previousLeft, previousRight, previousLb, previousRb;
    private FieldMap.Pose origin;
    private Pose goal;
    private double trialStarted;
    private double lastTime;
    private double settledSince = Double.NaN;
    private double coastStarted;
    private double speedSum, minSpeed, maxSpeed;
    private int speedSamples, coastSamples;
    private double sumTime, sumVelocity, sumTimeSquared, sumTimeVelocity;
    private String status = "Select a test; START does not start motion";
    private String forwardMeasurement = "Not measured";
    private String lateralMeasurement = "Not measured";

    @Override public void init() {
        profile = RobotProfile.loadActive(null);
        if (profile.navigation == null) {
            throw new IllegalStateException("Pedro tuning requires a navigation profile section");
        }
        drive.init(hardwareMap, profile);
        driveReady = true;
        stopDrive();
        tracking.init(hardwareMap, profile);
        follower = ProfiledPedroFollower.create(drive, tracking, profile, 1);
        RobotProfile.NavigationSettings n = profile.navigation;
        double[] initial = {0.2, 12, 30, n.translationalP, n.translationalD, n.translationalF,
                n.headingP, n.headingD, n.headingF, n.driveP, n.driveD, n.driveF,
                n.forwardVelocity, n.lateralVelocity, n.massKg,
                n.forwardZeroPowerAcceleration, n.lateralZeroPowerAcceleration, 1.5};
        System.arraycopy(initial, 0, values, 0, initial.length);
        for (Parameter p : Parameter.values()) {
            if (isTrialSetting(p) && profile.tuning.has(p.key)) {
                values[p.ordinal()] = profile.tuning.get(p.key).getAsDouble();
            }
        }
        tuningStore = new RobotProfileTuningStore(profile);
        publish();
    }

    @Override public void init_loop() {
        handleControls(false);
        stopDrive();
        publish();
    }

    @Override public void start() {
        try {
            drive.resetHeading(0);
            tracking.start(new FieldMap.Pose(0, 0, 0));
            started = true;
            lastTime = getRuntime();
            // Sample held buttons so START cannot accidentally launch a trial.
            sampleButtons();
            stopDrive();
            publish();
        } catch (RuntimeException exception) {
            cancel("Initialization failed");
            throw exception;
        }
    }

    @Override public void loop() {
        try {
            tracking.update(drive.getHeadingDegrees(), drive.getYawRateDegreesPerSecond());
            double now = getRuntime();
            if (!tracking.hasValidMotion()) {
                cancel("Tracking unavailable: STOP and reinitialize");
                sampleButtons();
            } else if (!Double.isFinite(now) || now < lastTime) {
                cancel("Invalid clock: STOP and reinitialize");
                sampleButtons();
            } else {
                handleControls(true);
                if (active) updateTrial(now);
                else stopDrive();
            }
            lastTime = now;
            publish();
        } catch (RuntimeException exception) {
            cancel("Tuning failure; drive stopped");
            throw exception;
        }
    }

    @Override public void stop() { cancel("OpMode stopped"); }

    private void handleControls(boolean allowMotion) {
        if (gamepad1.b || gamepad1.y) {
            cancel(gamepad1.y ? "Stopped by Y" : "Stopped by B");
            sampleButtons();
            return;
        }
        if (!active) {
            if (gamepad1.dpad_up && !previousUp) {
                mode = Mode.values()[(mode.ordinal() + 1) % Mode.values().length];
            }
            if (gamepad1.dpad_down && !previousDown) {
                mode = Mode.values()[(mode.ordinal() + Mode.values().length - 1) % Mode.values().length];
            }
            if (gamepad1.right_bumper && !previousRb) {
                parameter = Parameter.values()[(parameter.ordinal() + 1) % Parameter.values().length];
            }
            if (gamepad1.left_bumper && !previousLb) {
                parameter = Parameter.values()[(parameter.ordinal() + Parameter.values().length - 1)
                        % Parameter.values().length];
            }
            int adjustment = (gamepad1.dpad_right && !previousRight ? 1 : 0)
                    - (gamepad1.dpad_left && !previousLeft ? 1 : 0);
            if (adjustment != 0) {
                double next = Math.max(parameter.min, Math.min(parameter.max,
                        value(parameter) + adjustment * parameter.step));
                if (next != value(parameter)) {
                    values[parameter.ordinal()] = next;
                    try {
                        tuningStore.save(parameter.key, next, isTrialSetting(parameter));
                        persistenceStatus = "Saved " + parameter.key + " to controller profile";
                    } catch (IOException | IllegalArgumentException | IllegalStateException exception) {
                        persistenceStatus = "NOT SAVED: " + parameter.key + ": " + exception.getMessage();
                    }
                }
            }
            if (gamepad1.x && !previousX) direction = -direction;
            if (allowMotion && gamepad1.a && !previousA) beginTrial(getRuntime());
        }
        sampleButtons();
    }

    private void sampleButtons() {
        previousA = gamepad1.a;
        previousX = gamepad1.x;
        previousUp = gamepad1.dpad_up;
        previousDown = gamepad1.dpad_down;
        previousLeft = gamepad1.dpad_left;
        previousRight = gamepad1.dpad_right;
        previousLb = gamepad1.left_bumper;
        previousRb = gamepad1.right_bumper;
    }

    private void beginTrial(double now) {
        if (mode == Mode.LOCALIZATION) {
            status = "Push robot by hand; compare pose to tape measure. No motor commands.";
            return;
        }
        if (!tracking.isStationary(profile.navigation.stationarySpeedInPerSec,
                profile.navigation.stationaryTurnRateDegreesPerSec)) {
            status = "Trial refused: robot must be stationary";
            return;
        }
        origin = tracking.getPose();
        trialStarted = now;
        settledSince = Double.NaN;
        coasting = false;
        speedSamples = coastSamples = 0;
        speedSum = sumTime = sumVelocity = sumTimeSquared = sumTimeVelocity = 0;
        minSpeed = Double.POSITIVE_INFINITY;
        maxSpeed = 0;
        applySettings();
        double h = Math.toRadians(origin.headingDegrees);
        boolean left = isLeft();
        double distance = direction * value(Parameter.DISTANCE);
        goal = new Pose(origin.x + distance * (left ? -Math.sin(h) : Math.cos(h)),
                origin.y + distance * (left ? Math.cos(h) : Math.sin(h)), h);
        if (mode == Mode.HEADING) {
            goal = new Pose(origin.x, origin.y, h + Math.toRadians(direction * value(Parameter.ANGLE)));
            follower.holdPoint(goal);
        } else if (mode == Mode.POSITION_FORWARD || mode == Mode.POSITION_LEFT) {
            follower.holdPoint(goal);
        } else if (isPath()) {
            Path path = new Path(new BezierLine(new Pose(origin.x, origin.y, h), goal));
            path.setConstantHeadingInterpolation(h);
            follower.followPath(path, true);
        }
        follower.useHeading = mode == Mode.HEADING || isPath();
        follower.useTranslational = mode != Mode.HEADING;
        follower.useDrive = isPath();
        follower.useCentripetal = isPath();
        active = true;
        status = "Running " + mode;
    }

    private void applySettings() {
        follower.getConstants().setCoefficientsTranslationalPIDF(new PIDFCoefficients(
                value(Parameter.TRANSLATIONAL_P), 0, value(Parameter.TRANSLATIONAL_D),
                value(Parameter.TRANSLATIONAL_F)));
        follower.getConstants().setCoefficientsHeadingPIDF(new PIDFCoefficients(
                value(Parameter.HEADING_P), 0, value(Parameter.HEADING_D), value(Parameter.HEADING_F)));
        follower.getConstants().setCoefficientsDrivePIDF(new FilteredPIDFCoefficients(
                value(Parameter.DRIVE_P), 0, value(Parameter.DRIVE_D), 0, value(Parameter.DRIVE_F)));
        follower.getConstants().setMass(value(Parameter.MASS));
        follower.getConstants().setForwardZeroPowerAcceleration(value(Parameter.FORWARD_ACCEL));
        follower.getConstants().setLateralZeroPowerAcceleration(value(Parameter.LATERAL_ACCEL));
        follower.updateConstants();
        follower.setXVelocity(value(Parameter.FORWARD_SPEED));
        follower.setYVelocity(value(Parameter.LATERAL_SPEED));
        follower.setMaxPowerScaling(value(Parameter.POWER));
    }

    private void updateTrial(double now) {
        FieldMap.Pose pose = tracking.getPose();
        double displacement = Math.hypot(pose.x - origin.x, pose.y - origin.y);
        double rotation = angleError(pose.headingDegrees, origin.headingDegrees);
        if (now - trialStarted >= 10 || displacement >= 48 || Math.abs(rotation) >= 120) {
            cancel("Safety limit: 10 s / 48 in / 120 degrees; trial stopped");
            return;
        }
        if (mode == Mode.POWER_FORWARD || mode == Mode.POWER_LEFT || mode == Mode.POWER_TURN) {
            if (now - trialStarted >= 2) {
                cancel("Power pulse complete (2 s)");
            } else {
                double power = direction * value(Parameter.POWER);
                drive.driveRobotRelative(mode == Mode.POWER_FORWARD ? power : 0,
                        mode == Mode.POWER_LEFT ? -power : 0, mode == Mode.POWER_TURN ? -power : 0);
            }
        } else if (mode == Mode.MEASURE_FORWARD || mode == Mode.MEASURE_LEFT) {
            measure(now, rotation);
        } else {
            follower.update();
            double positionError = Math.hypot(goal.getX() - pose.x, goal.getY() - pose.y);
            double headingError = Math.abs(angleError(Math.toDegrees(goal.getHeading()), pose.headingDegrees));
            boolean arrived = (mode == Mode.HEADING
                    || positionError <= profile.navigation.arrivalToleranceIn)
                    && ((mode == Mode.POSITION_FORWARD || mode == Mode.POSITION_LEFT)
                    || headingError <= profile.navigation.headingToleranceDegrees);
            if (arrived && tracking.isStationary(profile.navigation.stationarySpeedInPerSec,
                    profile.navigation.stationaryTurnRateDegreesPerSec)) {
                if (Double.isNaN(settledSince)) settledSince = now;
                if (now - settledSince >= profile.navigation.settleSeconds) {
                    cancel("Trial reached tolerance and settled; repeat in both directions");
                }
            } else {
                settledSince = Double.NaN;
            }
        }
    }

    private void measure(double now, double rotation) {
        if (Math.abs(rotation) > 5) {
            cancel("Measurement rejected: more than 5 degrees of heading drift");
            return;
        }
        double h = Math.toRadians(origin.headingDegrees);
        double velocity = direction * (isLeft()
                ? -tracking.getVelocityX() * Math.sin(h) + tracking.getVelocityY() * Math.cos(h)
                : tracking.getVelocityX() * Math.cos(h) + tracking.getVelocityY() * Math.sin(h));
        double poweredSeconds = value(Parameter.MEASUREMENT_SECONDS);
        if (!coasting && now - trialStarted < poweredSeconds) {
            double power = direction * value(Parameter.POWER);
            drive.driveRobotRelative(isLeft() ? 0 : power, isLeft() ? -power : 0, 0);
            if (now - trialStarted >= poweredSeconds - Math.min(0.5, poweredSeconds / 3)) {
                speedSum += velocity;
                speedSamples++;
                minSpeed = Math.min(minSpeed, velocity);
                maxSpeed = Math.max(maxSpeed, velocity);
            }
            return;
        }
        stopDrive();
        if (!coasting) {
            coasting = true;
            coastStarted = now;
            status = "Measuring zero-power BRAKE deceleration";
        }
        double t = now - coastStarted;
        coastSamples++;
        sumTime += t;
        sumVelocity += velocity;
        sumTimeSquared += t * t;
        sumTimeVelocity += t * velocity;
        if (velocity <= 1 || t >= 2) {
            double denominator = coastSamples * sumTimeSquared - sumTime * sumTime;
            double acceleration = denominator > 0
                    ? (coastSamples * sumTimeVelocity - sumTime * sumVelocity) / denominator : Double.NaN;
            double speed = speedSamples > 0 ? speedSum / speedSamples : Double.NaN;
            String result;
            if (speedSamples < 3 || speed < 1 || maxSpeed - minSpeed > speed * 0.15) {
                result = "Rejected: no stable positive speed plateau; repeat with more clearance/power";
            } else if (coastSamples < 3 || t < 0.05 || !Double.isFinite(acceleration) || acceleration >= 0) {
                result = "Rejected: insufficient deceleration samples; repeat trial";
            } else {
                result = String.format(Locale.US, "%.2f in/s at %.0f%%; acceleration %.2f in/s^2%s",
                        speed, value(Parameter.POWER) * 100, acceleration,
                        value(Parameter.POWER) >= 0.999 ? " (full-power speed)" : " (NOT full-power speed)");
            }
            if (isLeft()) lateralMeasurement = result;
            else forwardMeasurement = result;
            cancel("Measurement finished; see result. Values NOT applied automatically.");
        }
    }

    private boolean isLeft() {
        return mode == Mode.POWER_LEFT || mode == Mode.MEASURE_LEFT
                || mode == Mode.POSITION_LEFT || mode == Mode.PATH_LEFT;
    }

    private boolean isPath() { return mode == Mode.PATH_FORWARD || mode == Mode.PATH_LEFT; }
    private double value(Parameter p) { return values[p.ordinal()]; }
    private static boolean isTrialSetting(Parameter p) {
        return p == Parameter.POWER || p == Parameter.DISTANCE || p == Parameter.ANGLE
                || p == Parameter.MEASUREMENT_SECONDS;
    }
    private static double angleError(double a, double b) {
        return a - b - 360 * Math.floor((a - b + 180) / 360);
    }

    private void cancel(String reason) {
        active = false;
        if (follower != null) follower.breakFollowing();
        stopDrive();
        status = reason;
    }

    private void stopDrive() { if (driveReady) drive.driveRobotRelative(0, 0, 0); }

    private void publish() {
        telemetry.clearAll();
        telemetry.addLine("Pedro Chassis Tuning - no mechanisms or mapped obstacle avoidance");
        telemetry.addData("Profile", profile.name);
        telemetry.addData("Test", "%s / %s", mode, direction > 0 ? "forward / left / CCW" : "back / right / CW");
        telemetry.addData("Selected", "%s = %.4f (step %.4f)", parameter.key, value(parameter), parameter.step);
        telemetry.addLine("Idle: D-pad up/down test; bumpers parameter; left/right value; X direction");
        telemetry.addLine("Release then A to run; B/Y cancel; START alone never moves");
        telemetry.addData("Trial power (restored on INIT)", "%.0f%% - check before A",
                value(Parameter.POWER) * 100);
        telemetry.addLine("Clear at least 48 in of travel plus stopping margin.");
        telemetry.addLine("Edits autosave to controller profile; uploads overwrite them. Copy to source.");
        telemetry.addLine("Tuning bypasses evidence flags; does not mark chassis verified.");
        telemetry.addLine(persistenceStatus);
        for (Parameter p : Parameter.values()) {
            telemetry.addData((p == parameter ? "> " : "") + p.key, "%.4f", value(p));
        }
        telemetry.addLine(status);
        if (started && tracking.getPose() != null) {
            FieldMap.Pose pose = tracking.getPose();
            telemetry.addData("Pose", "X %.2f Y %.2f H %.2f", pose.x, pose.y, pose.headingDegrees);
            telemetry.addData("Motion", "Vx %.2f Vy %.2f in/s; turn %.2f deg/s",
                    tracking.getVelocityX(), tracking.getVelocityY(), tracking.getHeadingRateDegreesPerSecond());
            if (active && goal != null && (isPath() || mode == Mode.HEADING
                    || mode == Mode.POSITION_FORWARD || mode == Mode.POSITION_LEFT)) {
                telemetry.addData("Target error", "%.2f in / %.2f deg",
                        Math.hypot(goal.getX() - pose.x, goal.getY() - pose.y),
                        angleError(Math.toDegrees(goal.getHeading()), pose.headingDegrees));
                telemetry.addData("Pedro", "stuck %s; busy %s; remaining %.2f in",
                        follower.isRobotStuck(), follower.isBusy(), follower.getTotalDistanceRemaining());
            }
        }
        telemetry.addData("Wheel powers FL/FR/BL/BR", "%.3f / %.3f / %.3f / %.3f",
                drive.getLeftFrontPower(), drive.getRightFrontPower(),
                drive.getLeftBackPower(), drive.getRightBackPower());
        telemetry.addData("Forward measurement", forwardMeasurement);
        telemetry.addData("Lateral measurement", lateralMeasurement);
        telemetry.addLine("Copy saved navigation values back to source before uploading (I stays 0):");
        telemetry.addData("translational P/D/F", "%.4f / %.4f / %.4f",
                value(Parameter.TRANSLATIONAL_P), value(Parameter.TRANSLATIONAL_D), value(Parameter.TRANSLATIONAL_F));
        telemetry.addData("heading P/D/F", "%.4f / %.4f / %.4f",
                value(Parameter.HEADING_P), value(Parameter.HEADING_D), value(Parameter.HEADING_F));
        telemetry.addData("drive P/D/F", "%.4f / %.4f / %.4f",
                value(Parameter.DRIVE_P), value(Parameter.DRIVE_D), value(Parameter.DRIVE_F));
        telemetry.addData("forwardVelocity / lateralVelocity / massKg", "%.2f / %.2f / %.2f",
                value(Parameter.FORWARD_SPEED), value(Parameter.LATERAL_SPEED), value(Parameter.MASS));
        telemetry.addData("forward / lateral ZeroPowerAcceleration", "%.2f / %.2f",
                value(Parameter.FORWARD_ACCEL), value(Parameter.LATERAL_ACCEL));
        telemetry.update();
    }
}
