package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.control.DriveModePreferences;
import org.firstinspires.ftc.teamcode.control.LandmarkFieldMap;
import org.firstinspires.ftc.teamcode.control.LandmarkNavigationService;
import org.firstinspires.ftc.teamcode.control.LandmarkTravelControls;
import org.firstinspires.ftc.teamcode.subsystems.ConfigurationSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FieldMap;
import org.firstinspires.ftc.teamcode.subsystems.RobotProfile;
import org.firstinspires.ftc.teamcode.subsystems.TrackingSubsystem;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/** Autonomous-only landmark navigation experiment with the existing manual drive as fallback. */
@TeleOp(name = "Landmark Travel Test", group = "2 - (Tests)")
public final class LandmarkTravelTestTeleOp extends OpMode {
    private final DriveSubsystem drive = new DriveSubsystem();
    private final ConfigurationSubsystem configuration = new ConfigurationSubsystem();
    private final TrackingSubsystem tracking = new TrackingSubsystem();
    private final LandmarkTravelControls controls = new LandmarkTravelControls();
    private RobotProfile profile;
    private DriveModePreferences preferences;
    private LandmarkFieldMap fieldMap;
    private LandmarkNavigationService navigation;
    private final List<LandmarkFieldMap.Destination> destinations = new ArrayList<>();
    private FieldMap.Pose savedHome;
    private int selected;
    private boolean started;
    private boolean driveReady;
    private String controlStatus = "Initialize the robot";

    @Override public void init() {
        profile = RobotProfile.loadActive(null);
        fieldMap = new LandmarkFieldMap(profile);
        drive.init(hardwareMap, profile);
        driveReady = true;
        drive.driveRobotRelative(0, 0, 0);
        tracking.init(hardwareMap, profile);
        configuration.init(telemetry, gamepad1, new DriveModePreferences(hardwareMap.appContext));
        preferences = new DriveModePreferences(hardwareMap.appContext);
        configuration.setRobotProfileName(profile.name);
        destinations.addAll(fieldMap.destinations());
        savedHome = loadSafeHome();
        if (savedHome != null) destinations.add(LandmarkFieldMap.Destination.savedHome(savedHome));
        navigation = new LandmarkNavigationService(drive, tracking, profile);
        publish(null);
    }

    @Override public void init_loop() {
        configuration.updateInit(gamepad1);
        publish(null);
    }

    @Override public void start() {
        if (navigation == null) return;
        configuration.lock();
        FieldMap.Pose initial = FieldMap.startingPose(configuration.getAlliance(),
                configuration.getStartingSpot(), profile.chassisLengthCm / 2.54);
        drive.resetHeading(initial.headingDegrees,
                configuration.getFieldForwardHeadingDegrees());
        tracking.start(initial);
        LandmarkTravelControls.Input input = readControls();
        controls.seed(input);
        started = true;
        controlStatus = profile.isAutonomousReady()
                ? "Autonomous ready; manual driving available"
                : "Physical autonomous blocked until geometry and Pedro tuning are verified";
    }

    @Override public void loop() {
        if (!started) return;
        tracking.update(drive.getHeadingDegrees(), drive.getYawRateDegreesPerSecond());
        LandmarkTravelControls.Actions actions = controls.update(readControls(), navigation.isActive());
        if (actions.selectionDelta != 0) {
            selected = (selected + actions.selectionDelta + destinations.size()) % destinations.size();
        }

        if (actions.stop) {
            navigation.cancel(gamepad1.y ? "Stopped by Y" : gamepad1.b ? "Stopped by B"
                    : actions.calibrate ? "Calibration requested" : "Home saved");
            stopDrive();
            if (actions.calibrate) {
                boolean calibrated = tracking.calibrateImuIfStationary(
                        profile.navigation.stationarySpeedInPerSec,
                        profile.navigation.stationaryTurnRateDegreesPerSec);
                controlStatus = calibrated ? "OTOS IMU calibrated; field coordinates unchanged"
                        : "Calibration refused: tracking must be valid and stationary";
            } else if (actions.saveHome) {
                saveHome();
            } else {
                controlStatus = gamepad1.y ? "Stopped by Y" : "Stopped by B";
            }
        } else if (actions.toggleRoute) {
            if (navigation.isActive()) {
                navigation.cancel("Route cancelled by A");
                stopDrive();
            } else {
                boolean accepted = navigation.start(destinations.get(selected), getRuntime());
                controlStatus = accepted ? "Route started" : navigation.getStatus();
            }
        } else if (actions.manual) {
            navigation.cancel("Manual override");
            if (!Double.isFinite(drive.getHeadingDegrees())) {
                stopDrive();
                controlStatus = "Manual drive stopped: IMU heading unavailable";
                publish(tracking.getPose());
                return;
            }
            if (configuration.isFieldCentric()) {
                drive.driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x,
                        gamepad1.right_stick_x, profile.navigation.transitPower);
            } else {
                drive.driveRobotRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x,
                        gamepad1.right_stick_x, profile.navigation.transitPower);
            }
            controlStatus = String.format(Locale.US, "Manual drive at %.0f%%",
                    profile.navigation.transitPower * 100);
        } else if (navigation.isActive()) {
            navigation.update(getRuntime());
            controlStatus = navigation.getStatus();
        } else {
            stopDrive();
        }
        publish(tracking.getPose());
    }

    @Override public void stop() {
        if (navigation != null) navigation.cancel("OpMode stopped");
        stopDrive();
    }

    private void saveHome() {
        FieldMap.Pose pose = tracking.getPose();
        if (!tracking.hasValidPose() || pose == null || !fieldMap.isSafeHome(pose)) {
            controlStatus = "Home refused: tracked pose is invalid or outside safe mapped space";
            return;
        }
        try {
            fieldMap.plan(pose, LandmarkFieldMap.Destination.savedHome(pose));
            preferences.saveNavigationHome(pose);
            savedHome = pose;
            if (destinations.size() == 12) {
                destinations.add(LandmarkFieldMap.Destination.savedHome(savedHome));
            } else {
                destinations.set(12, LandmarkFieldMap.Destination.savedHome(savedHome));
            }
            controlStatus = String.format(Locale.US, "Saved safe home at X %.1f Y %.1f H %.1f",
                    pose.x, pose.y, pose.headingDegrees);
        } catch (IllegalArgumentException | IllegalStateException exception) {
            controlStatus = "Home refused: " + exception.getMessage();
        }
    }

    private FieldMap.Pose loadSafeHome() {
        FieldMap.Pose candidate = preferences.getNavigationHome();
        if (candidate == null) return null;
        if (fieldMap.isSafeHome(candidate)) return candidate;
        controlStatus = "Saved home ignored: profile geometry marks it unsafe";
        return null;
    }

    private LandmarkTravelControls.Input readControls() {
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

    private void publish(FieldMap.Pose pose) {
        telemetry.clearAll();
        configuration.publish();
        telemetry.addLine("");
        telemetry.addLine("Landmark travel: d-pad up/down select; A starts/cancels route.");
        telemetry.addLine("B/Y stop. Sticks cancel to manual at 50%. X calibrates OTOS IMU.");
        telemetry.addLine("Options/Start saves a safe home. No vision or mechanism actuation.");
        telemetry.addData("Selected destination", "%s (%d / %d)",
                destinations.isEmpty() ? "Unavailable" : destinations.get(selected).name,
                Math.min(selected + 1, destinations.size()), destinations.size());
        for (int row = 0; row < destinations.size(); row++) {
            telemetry.addLine((row == selected ? "> " : "  ") + destinations.get(row).name);
        }
        telemetry.addData("Active target", navigation == null || navigation.getTarget() == null
                ? "None" : navigation.getTarget().name);
        telemetry.addData("Robot pose", pose == null ? "Unavailable"
                : String.format(Locale.US, "X %.1f Y %.1f H %.1f",
                        pose.x, pose.y, pose.headingDegrees));
        telemetry.addData("Navigation phase", navigation == null ? "Not initialized"
                : navigation.getPhase());
        telemetry.addData("Power limit", navigation == null ? "0%"
                : String.format(Locale.US, "%.0f%%", navigation.getPowerLimit() * 100));
        telemetry.addData("Readiness", readiness());
        telemetry.addData("Control status", controlStatus);
        telemetry.addData("Navigation status", navigation == null
                ? "Not initialized" : navigation.getStatus());
        if (tracking != null && started) {
            telemetry.addData("Tracking", tracking.getStatus());
            telemetry.addData("Measured motion", "%.2f in/s, %.1f deg/s",
                    tracking.getSpeedInchesPerSecond(),
                    tracking.getHeadingRateDegreesPerSecond());
        }
        telemetry.update();
    }

    private String readiness() {
        if (profile == null) return "Profile unavailable";
        if (profile.isSimulatorRuntime()) {
            return profile.isAutonomousReady() ? "Simulator experiment enabled"
                    : "Simulator experiment disabled by profile";
        }
        return profile.isAutonomousReady() ? "Physical geometry and Pedro tuning marked verified"
                : "Blocked: physical geometry and Pedro tuning are unverified";
    }

    private void stopDrive() {
        if (driveReady) drive.driveRobotRelative(0, 0, 0);
    }
}
