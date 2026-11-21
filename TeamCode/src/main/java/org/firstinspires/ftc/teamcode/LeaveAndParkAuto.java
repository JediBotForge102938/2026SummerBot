package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.control.DriveModePreferences;
import org.firstinspires.ftc.teamcode.control.LandmarkFieldMap;
import org.firstinspires.ftc.teamcode.control.LandmarkNavigationService;
import org.firstinspires.ftc.teamcode.subsystems.ConfigurationSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FieldMap;
import org.firstinspires.ftc.teamcode.subsystems.RobotProfile;
import org.firstinspires.ftc.teamcode.subsystems.TrackingSubsystem;

/** Travels once to the selected alliance's parking approach, then leaves drive outputs stopped. */
@Autonomous(name = "Leave & Park", group = "1 - (Autonomous)")
public final class LeaveAndParkAuto extends OpMode {
    private final ConfigurationSubsystem configuration = new ConfigurationSubsystem();
    private final DriveSubsystem drive = new DriveSubsystem();
    private final TrackingSubsystem tracking = new TrackingSubsystem();
    private RobotProfile profile;
    private LandmarkFieldMap map;
    private LandmarkNavigationService navigation;
    private boolean driveReady;
    private boolean started;

    @Override public void init() {
        profile = RobotProfile.loadActive(null);
        map = new LandmarkFieldMap(profile);
        drive.init(hardwareMap, profile);
        driveReady = true;
        stopDrive();
        tracking.init(hardwareMap, profile);
        configuration.init(telemetry, gamepad1, new DriveModePreferences(hardwareMap.appContext));
        configuration.setRobotProfileName(profile.name);
        navigation = new LandmarkNavigationService(drive, tracking, profile);
        publish();
    }

    @Override public void init_loop() {
        configuration.updateInit(gamepad1);
        publish();
    }

    @Override public void start() {
        configuration.lock();
        FieldMap.Pose initial = FieldMap.startingPose(configuration.getAlliance(),
                configuration.getStartingSpot(), profile.chassisLengthCm / 2.54);
        drive.resetHeading(initial.headingDegrees, configuration.getFieldForwardHeadingDegrees());
        tracking.start(initial);
        tracking.update(drive.getHeadingDegrees(), drive.getYawRateDegreesPerSecond());
        started = true;
        navigation.start(map.parkingDestination(configuration.getAlliance(),
                configuration.getStartingSpot()), getRuntime());
        publish();
    }

    @Override public void loop() {
        if (!started) return;
        if (gamepad1.y || gamepad1.b) {
            navigation.cancel(gamepad1.y ? "Stopped by Y" : "Stopped by B");
            stopDrive();
        } else if (navigation.isActive()) {
            tracking.update(drive.getHeadingDegrees(), drive.getYawRateDegreesPerSecond());
            navigation.update(getRuntime());
        } else {
            stopDrive();
        }
        publish();
    }

    @Override public void stop() {
        if (navigation != null) navigation.cancel("OpMode stopped");
        stopDrive();
    }

    private void stopDrive() {
        if (driveReady) drive.driveRobotRelative(0, 0, 0);
    }

    private void publish() {
        telemetry.clearAll();
        if (!started) configuration.publish();
        telemetry.addLine("Leave & Park");
        telemetry.addData("Destination", map.parkingDestination(configuration.getAlliance(),
                configuration.getStartingSpot()).name);
        telemetry.addData("Phase", navigation.getPhase());
        telemetry.addLine(navigation.getStatus());
        if (!profile.isAutonomousReady()) {
            telemetry.addLine("Autonomous blocked: profile geometry/tuning not verified or simulator disabled");
        }
        FieldMap.Pose pose = tracking.getPose();
        if (pose != null) {
            telemetry.addData("Pose", "X %.1f Y %.1f H %.1f",
                    pose.x, pose.y, pose.headingDegrees);
        }
        telemetry.update();
    }
}
