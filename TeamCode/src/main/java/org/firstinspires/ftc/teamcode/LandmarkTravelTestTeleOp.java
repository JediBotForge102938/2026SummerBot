package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.Paths;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.control.BioBuzzFieldMap;
import org.firstinspires.ftc.teamcode.control.BioBuzzFieldMap.Destination;
import org.firstinspires.ftc.teamcode.control.DriveModePreferences;
import org.firstinspires.ftc.teamcode.control.LandmarkTravelControls;
import org.firstinspires.ftc.teamcode.pedro.PedroConstants;
import org.firstinspires.ftc.teamcode.subsystems.ConfigurationSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FieldMap;
import org.firstinspires.ftc.teamcode.subsystems.RobotProfile;

@TeleOp(name = "Landmark Travel Test", group = "2 - (Tests)")
public final class LandmarkTravelTestTeleOp extends OpMode {
    private static final double MANUAL_POWER = 0.5;
    private static final double STATIONARY_SPEED_IN_PER_SEC = 1.5;
    private static final double STATIONARY_TURN_DEG_PER_SEC = 5;

    private final ConfigurationSubsystem configuration = new ConfigurationSubsystem();
    private final LandmarkTravelControls controls = new LandmarkTravelControls();
    private final List<Destination> destinations = new ArrayList<>();
    private RobotProfile profile;
    private Follower follower;
    private SparkFunOTOS otos;
    private BioBuzzFieldMap field;
    private Destination target;
    private String routeStatus = "Ready";
    private String controlStatus = "Initialize the robot";
    private double peakEstimatedPathError;
    private int selected;
    private boolean started;

    @Override public void init() {
        profile = RobotProfile.loadActive(null);
        configuration.init(telemetry, gamepad1, new DriveModePreferences(hardwareMap.appContext));
        configuration.setRobotProfileName(profile.name);
        follower = PedroConstants.create(hardwareMap);
        otos = hardwareMap.get(SparkFunOTOS.class, PedroConstants.otosConfig(profile).name.get());
        field = new BioBuzzFieldMap(profile.chassisLengthCm / 2.54, profile.chassisWidthCm / 2.54);
        destinations.addAll(field.destinations());
        publish();
    }

    @Override public void init_loop() {
        configuration.updateInit(gamepad1, gamepad2);
        publish();
    }

    @Override public void start() {
        configuration.lock();
        follower.setPose(pose(field.startingPose(configuration.getAlliance(),
                configuration.getStartingSpot())));
        controls.seed(input());
        started = true;
        controlStatus = "Autonomous ready; manual driving available";
    }

    @Override public void loop() {
        LandmarkTravelControls.Actions actions = controls.update(input(), follower.following());
        if (actions.selectionDelta != 0) {
            selected = (selected + actions.selectionDelta + destinations.size()) % destinations.size();
        }
        if (actions.stop) {
            follower.stop();
            follower.drivetrain.stop();
            routeStatus = gamepad1.y ? "Stopped by Y" : gamepad1.b ? "Stopped by B"
                    : actions.calibrate ? "Calibration requested" : "Home saved";
            if (actions.calibrate) {
                Velocity motion = follower.velocity();
                if (!finite(motion) || Math.hypot(motion.vx, motion.vy) > STATIONARY_SPEED_IN_PER_SEC
                        || Math.abs(Math.toDegrees(motion.omega)) > STATIONARY_TURN_DEG_PER_SEC) {
                    controlStatus = "Calibration refused: tracking must be valid and stationary";
                } else {
                    controlStatus = otos.calibrateImu()
                            ? "OTOS IMU calibrated; field coordinates unchanged"
                            : "OTOS IMU calibration failed";
                }
            } else if (actions.saveHome) {
                saveHome();
            } else {
                controlStatus = routeStatus;
            }
        } else if (actions.manual) {
            DrivePowers powers = new DrivePowers(-gamepad1.left_stick_y * MANUAL_POWER,
                    gamepad1.left_stick_x * MANUAL_POWER, gamepad1.right_stick_x * MANUAL_POWER);
            if (configuration.isFieldCentric()) {
                powers = ManualDrive.fieldCentric(powers, follower.pose().heading());
            }
            follower.manual(powers);
            routeStatus = "Manual override";
            controlStatus = "Manual drive at 50%";
        } else if (actions.toggleRoute) {
            if (follower.following()) {
                follower.stop();
                routeStatus = "Route cancelled by A";
                controlStatus = routeStatus;
            } else {
                follow(destinations.get(selected));
            }
        } else if (follower.manual()) {
            follower.stop();
        }
        follower.update();
        if (follower.following()) {
            Pose current = follower.pose();
            Pose closest = follower.closestPose();
            peakEstimatedPathError = Math.max(peakEstimatedPathError,
                    Math.hypot(current.x() - closest.x(), current.y() - closest.y()));
        } else if (routeStatus.equals("Following planned route")) {
            routeStatus = follower.holding() ? "Holding destination; verify pose"
                    : "Route ended; verify pose";
        }
        publish();
    }

    private void follow(Destination destination) {
        Pose from = follower.pose();
        try {
            List<FieldMap.Pose> waypoints = field.plan(fieldPose(from), destination.pose);
            target = destination;
            if (waypoints.isEmpty()) {
                routeStatus = "Already at destination";
            } else {
                List<Path> segments = new ArrayList<>();
                for (FieldMap.Pose waypoint : waypoints) {
                    Pose to = pose(waypoint);
                    segments.add(Paths.line(from, to).linear(from, to));
                    from = to;
                }
                peakEstimatedPathError = 0;
                follower.follow(Paths.path(segments.toArray(new Path[0])));
                routeStatus = "Following planned route";
            }
            controlStatus = routeStatus;
        } catch (IllegalArgumentException exception) {
            follower.stop();
            follower.drivetrain.stop();
            routeStatus = "Route refused: " + exception.getMessage();
            controlStatus = routeStatus;
        }
    }

    private void saveHome() {
        try {
            FieldMap.Pose home = fieldPose(follower.pose());
            field.plan(home, home);
            Destination saved = new Destination("Saved home", home);
            if (destinations.size() == 12) destinations.add(saved);
            else destinations.set(12, saved);
            controlStatus = String.format(Locale.US, "Saved safe home at X %.1f Y %.1f H %.1f",
                    home.x, home.y, home.headingDegrees);
        } catch (IllegalArgumentException exception) {
            routeStatus = "Home refused: " + exception.getMessage();
            controlStatus = routeStatus;
        }
    }

    @Override public void stop() {
        if (follower != null) {
            follower.stop();
            follower.drivetrain.stop();
        }
    }

    private LandmarkTravelControls.Input input() {
        LandmarkTravelControls.Input input = new LandmarkTravelControls.Input();
        input.a = gamepad1.a;
        input.b = gamepad1.b;
        input.x = gamepad1.x;
        input.y = gamepad1.y;
        input.options = gamepad1.options;
        input.start = gamepad1.start;
        input.dpadUp = gamepad1.dpad_up;
        input.dpadDown = gamepad1.dpad_down;
        input.leftX = gamepad1.left_stick_x;
        input.leftY = gamepad1.left_stick_y;
        input.rightX = gamepad1.right_stick_x;
        return input;
    }

    private static Pose pose(FieldMap.Pose p) {
        return new Pose(p.x, p.y, Math.toRadians(p.headingDegrees));
    }

    private static FieldMap.Pose fieldPose(Pose p) {
        return new FieldMap.Pose(p.x(), p.y(), Math.toDegrees(p.heading()));
    }

    private static boolean finite(Velocity motion) {
        return motion != null && Double.isFinite(motion.vx)
                && Double.isFinite(motion.vy) && Double.isFinite(motion.omega);
    }

    private void publish() {
        telemetry.clearAll();
        if (!started) {
            configuration.publish();
            telemetry.addData("Place robot at", pose(field.startingPose(
                    configuration.getAlliance(), configuration.getStartingSpot())).toString());
            telemetry.addLine("");
        }
        telemetry.addLine("Navigation");
        telemetry.addData("Target", target == null ? "None" : target.name);
        telemetry.addData("Phase", follower.mode().toString());
        Pose current = follower.pose();
        telemetry.addData("Pose", !started ? "Unavailable"
                : String.format(Locale.US, "X %.1f Y %.1f H %.1f",
                        current.x(), current.y(), Math.toDegrees(current.heading())));
        if (target != null) {
            telemetry.addData("Goal", "X %.1f Y %.1f H %.1f",
                    target.pose.x, target.pose.y, target.pose.headingDegrees);
        }
        telemetry.addData("Power", follower.following() ? "Pedro controlled"
                : follower.manual() ? "50% manual" : "0%");
        telemetry.addLine("Status");
        publishWrapped(routeStatus);
        telemetry.addLine("");
        telemetry.addLine("Landmarks");
        for (int row = 0; row < destinations.size(); row++) {
            telemetry.addLine((row == selected ? "> " : "  ") + destinations.get(row).name);
        }
        telemetry.addLine("");
        publishWrapped(profile.isSimulatorRuntime() ? "Simulator experiment enabled"
                : "Physical tuning required; verify clearance");
        publishWrapped(controlStatus);
        if (started) {
            Velocity motion = follower.velocity();
            telemetry.addData("Tracking", finite(motion) ? "OTOS" : "Motion unavailable");
            if (finite(motion)) {
                telemetry.addData("Measured motion", "%.2f in/s, %.1f deg/s",
                        Math.hypot(motion.vx, motion.vy), Math.toDegrees(motion.omega));
                SparkFunOTOS.Pose2D raw = otos.getVelocity();
                telemetry.addData("OTOS raw velocity", "X %.2f Y %.2f in/s",
                        DistanceUnit.INCH.fromUnit(otos.getLinearUnit(), raw.x),
                        DistanceUnit.INCH.fromUnit(otos.getLinearUnit(), raw.y));
            } else {
                telemetry.addData("OTOS raw velocity", "Unavailable (tracking invalid)");
            }
            telemetry.addData("Peak estimated path error (in)", "%.2f", peakEstimatedPathError);
        }
        telemetry.update();
    }

    private void publishWrapped(String text) {
        final int width = 42;
        while (text.length() > width) {
            int split = text.lastIndexOf(' ', width);
            if (split <= 0) split = width;
            telemetry.addLine(text.substring(0, split));
            text = text.substring(split).trim();
        }
        telemetry.addLine(text);
    }
}
