package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.BioBuzzLimelightClusterAdapter.ClusterDetection;
import org.firstinspires.ftc.teamcode.control.DriveModePreferences;
import org.firstinspires.ftc.teamcode.control.ImuAimController;
import org.firstinspires.ftc.teamcode.control.HiveTrackingController;
import org.firstinspires.ftc.teamcode.control.LandmarkFieldMap;
import org.firstinspires.ftc.teamcode.control.LandmarkNavigationService;
import org.firstinspires.ftc.teamcode.subsystems.ConfigurationSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ConfigurationSubsystem.StartingSpot;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FieldMap;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherFeeder;
import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotProfile;
import org.firstinspires.ftc.teamcode.subsystems.TrackingSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WindmillSubsystem;

@Autonomous(name = "Auto Shoot", group = "1 - (Autonomous)")
public final class AutoShoot extends OpMode {
    // Timed estimates, not ball counts. Calibrate with four pollen on the actual robot.
    static final double BATCH_FEED_SECONDS = 2.6;
    static final double COLLECTION_SECONDS = 2.0;
    static final double GARDEN_PRESS_POWER = 0.15;
    static final double TURN_RESPONSE_SCALE = 1.5;
    static final double HIVE_SETTLE_SECONDS = 0.6;
    static final double AIM_DWELL_SECONDS = 0.25;
    static final double VISION_TIMEOUT_SECONDS = 4.0;
    static final double SHOOT_TIMEOUT_SECONDS = 8.0;
    static final double AUTO_SECONDS = 30.0;

    enum Phase { IDLE, DRIVE_PRELOAD, AIM_PRELOAD, SHOOT_PRELOAD, CHECK_HIVE,
        DRIVE_GARDEN, COLLECT, DRIVE_COLLECTED, AIM_COLLECTED, SHOOT_COLLECTED, DRIVE_PARK,
        DONE, STOPPED }

    private final ConfigurationSubsystem configuration = new ConfigurationSubsystem();
    private final DriveSubsystem drive = new DriveSubsystem();
    private final TrackingSubsystem tracking = new TrackingSubsystem();
    private final VisionSubsystem vision = new VisionSubsystem();
    private RobotProfile profile;
    private LandmarkFieldMap map;
    private LandmarkNavigationService navigation;
    private ImuAimController aim;
    private IntakeSubsystem intake;
    private LauncherSubsystem launcher;
    private LauncherFeeder feeder;
    private boolean driveReady, intakeReady, launcherReady, feederReady;
    private boolean started;
    private Phase phase = Phase.IDLE;
    private StartingSpot hiveSide;
    private double autoStartedAt, phaseStartedAt, lastLoopAt;
    private double alignedSince = Double.NaN;
    private double feedSeconds;
    private boolean feeding;
    private String status = "Select alliance and starting spot during INIT";
    private String hiveStatus = "Not observed";
    private String parkingReason;

    @Override public void init() {
        profile = RobotProfile.loadActive(null);
        if (!"starterbot".equals(profile.robot) && !"competitionbot".equals(profile.robot)) {
            throw new IllegalStateException("Auto Shoot requires StarterBot or CompetitionBot");
        }
        boolean starter = "starterbot".equals(profile.robot);
        if (!starter) {
            GateSubsystem gate = new GateSubsystem(profile.launcherGate);
            gate.init(hardwareMap);
            feeder = gate;
            feederReady = true;
        }
        intake = new IntakeSubsystem(starter, profile);
        launcher = new LauncherSubsystem(profile.launcherMotors, profile.launcherPidf,
                profile.launcherFeedVelocityMargin);
        drive.init(hardwareMap, profile);
        driveReady = true;
        stopDrive();
        intake.init(hardwareMap);
        intakeReady = true;
        launcher.init(hardwareMap);
        launcherReady = true;
        if (starter) {
            WindmillSubsystem windmill = new WindmillSubsystem();
            windmill.init(hardwareMap);
            feeder = windmill;
        }
        feederReady = true;
        tracking.init(hardwareMap, profile);
        vision.init(hardwareMap, profile);
        configuration.init(telemetry, gamepad1, new DriveModePreferences(hardwareMap.appContext));
        configuration.setRobotProfileName(profile.name);
        map = new LandmarkFieldMap(profile);
        navigation = new LandmarkNavigationService(drive, tracking, profile, TURN_RESPONSE_SCALE);
        aim = new ImuAimController(drive::getYawRateDegreesPerSecond);
        publish();
    }

    @Override public void init_loop() {
        configuration.updateInit(gamepad1);
        publish();
    }

    @Override public void start() {
        configuration.lock();
        autoStartedAt = lastLoopAt = getRuntime();
        started = true;
        if (!profile.isAutonomousReady()) {
            halt("Autonomous blocked: profile geometry/tuning not verified or simulator disabled");
            publish();
            return;
        }
        FieldMap.Pose initial = FieldMap.startingPose(configuration.getAlliance(),
                configuration.getStartingSpot(), profile.chassisLengthCm / 2.54);
        drive.resetHeading(initial.headingDegrees, configuration.getFieldForwardHeadingDegrees());
        tracking.start(initial);
        tracking.update(drive.getHeadingDegrees(), drive.getYawRateDegreesPerSecond());
        hiveSide = configuration.getStartingSpot();
        driveToHive(false, autoStartedAt);
        publish();
    }

    @Override public void loop() {
        if (!started || phase == Phase.STOPPED || phase == Phase.DONE) return;
        double now = getRuntime();
        if (!Double.isFinite(now) || now < lastLoopAt || now - autoStartedAt >= AUTO_SECONDS
                || gamepad1.b || gamepad1.y) {
            halt(gamepad1.b || gamepad1.y ? "Stopped by driver" : "Autonomous clock/deadline stop");
            publish();
            return;
        }
        if (feeding) feedSeconds += now - lastLoopAt;
        lastLoopAt = now;
        tracking.update(drive.getHeadingDegrees(), drive.getYawRateDegreesPerSecond());
        if (!tracking.hasValidMotion()) {
            halt("Tracking lost: " + tracking.getStatus());
            publish();
            return;
        }
        aim.sample();
        switch (phase) {
            case DRIVE_PRELOAD:
            case DRIVE_GARDEN:
            case DRIVE_COLLECTED:
            case DRIVE_PARK:
                updateNavigation(now);
                break;
            case AIM_PRELOAD:
            case SHOOT_PRELOAD:
            case AIM_COLLECTED:
            case SHOOT_COLLECTED:
                updateShooting(now);
                break;
            case CHECK_HIVE:
                checkHiveAfterCollection(now);
                break;
            case COLLECT:
                updateCollection(now);
                break;
            default:
                halt("Unexpected autonomous phase");
        }
        publish();
    }

    private void updateNavigation(double now) {
        navigation.update(now);
        if (navigation.isActive()) return;
        stopDrive();
        if (!"Arrived".equals(navigation.getPhase())) {
            halt(navigation.getStatus());
        } else if (phase == Phase.DRIVE_GARDEN) {
            enter(Phase.COLLECT, now, "Low-power wall press while collecting");
            intake.setPower(1);
            updateCollection(now);
        } else if (phase == Phase.DRIVE_PARK) {
            enter(Phase.DONE, now, "Parked: " + parkingReason);
        } else {
            enter(phase == Phase.DRIVE_PRELOAD ? Phase.AIM_PRELOAD : Phase.AIM_COLLECTED,
                    now, "Waiting for scorable AprilTags and stable aim");
        }
    }

    private void updateCollection(double now) {
        if (now - phaseStartedAt >= COLLECTION_SECONDS) {
            stopDrive();
            stopMechanisms();
            feedSeconds = 0;
            enter(Phase.CHECK_HIVE, now, "Checking AprilTags from garden after collection");
            return;
        }
        FieldMap.Pose pose = tracking.getPose();
        FieldMap.Pose approach = map.gardenCollectionDestination(
                configuration.getAlliance()).robotPose;
        double headingError = pose == null ? Double.NaN
                : Math.IEEEremainder(pose.headingDegrees - approach.headingDegrees, 360);
        double wallLimit = FieldMap.HALF_FIELD - profile.chassisLengthCm / 2 / 2.54;
        if (pose == null || !Double.isFinite(headingError)
                || Math.abs(headingError) > profile.navigation.headingToleranceDegrees
                || Math.abs(pose.x - approach.x) > profile.navigation.arrivalToleranceIn + 0.5
                || Math.signum(pose.y) != Math.signum(approach.y)
                || Math.abs(pose.y) < Math.abs(approach.y) - profile.navigation.arrivalToleranceIn
                || Math.abs(pose.y) > wallLimit + profile.navigation.startWallPlacementToleranceIn) {
            halt("Garden wall press stopped: outside aligned collection corridor");
            return;
        }
        drive.driveRobotRelative(GARDEN_PRESS_POWER, 0, 0);
    }

    private void updateShooting(double now) {
        boolean collected = phase == Phase.AIM_COLLECTED || phase == Phase.SHOOT_COLLECTED;
        if (feedSeconds >= BATCH_FEED_SECONDS) {
            stopMechanisms();
            stopDrive();
            if (collected) {
                park("Completed timed garden batch", now);
            } else {
                startNavigation(map.gardenCollectionDestination(configuration.getAlliance()),
                        Phase.DRIVE_GARDEN, now);
                if (phase == Phase.DRIVE_GARDEN) intake.setPower(1);
            }
            return;
        }
        boolean shooting = phase == Phase.SHOOT_PRELOAD || phase == Phase.SHOOT_COLLECTED;
        if (now - phaseStartedAt >= (shooting ? SHOOT_TIMEOUT_SECONDS : VISION_TIMEOUT_SECONDS)) {
            String reason = shooting ? "Shooting timed out before batch finished"
                    : collected ? "No stable, scorable AprilTag aim before timeout"
                    : "Preload launcher not ready or robot not stationary before timeout";
            if (collected) park(reason, now);
            else halt(reason);
            return;
        }
        if (!collected) {
            stopDrive();
            boolean ready = launcher.update(true);
            boolean stationary = tracking.isStationary(profile.navigation.stationarySpeedInPerSec,
                    profile.navigation.stationaryTurnRateDegreesPerSec);
            if (!stationary) alignedSince = Double.NaN;
            else if (Double.isNaN(alignedSince)) alignedSince = now;
            boolean feed = ready && stationary && now - alignedSince >= AIM_DWELL_SECONDS;
            setFeeding(feed);
            if (feed && !shooting) {
                enter(Phase.SHOOT_PRELOAD, now, "Timed four-pollen preload batch at preset position");
            }
            return;
        }
        VisionSubsystem.DetectionSnapshot snapshot = vision.update(configuration.getAlliance());
        ClusterDetection target = findCell(snapshot);
        VisionSubsystem.CellStatus cellStatus = target == null
                ? VisionSubsystem.CellStatus.UNKNOWN_POSE : VisionSubsystem.classifyCell(target);
        hiveStatus = hiveSide + ": " + cellStatus;
        if (collected && cellStatus != VisionSubsystem.CellStatus.SCORABLE) {
            park("Selected hive missing or unscorable", now);
            return;
        }
        double error = Math.toDegrees(Math.atan2(target.poseCameraSpace.getPosition().x,
                target.poseCameraSpace.getPosition().z));
        double turn = aim.turn(error, snapshot.observationAgeMs);
        FieldMap.Pose pose = tracking.getPose();
        if (!map.isTransitClear(pose)) {
            halt("Shooting stopped: robot left clear turning space");
            return;
        }
        drive.driveRobotRelative(0, 0, turn);
        boolean ready = launcher.update(true);
        boolean aligned = Math.abs(aim.getCorrectedAngle()) <= 2.0
                && tracking.isStationary(profile.navigation.stationarySpeedInPerSec,
                        profile.navigation.stationaryTurnRateDegreesPerSec);
        if (!aligned) alignedSince = Double.NaN;
        else if (Double.isNaN(alignedSince)) alignedSince = now;
        boolean feed = ready && aligned && now - alignedSince >= AIM_DWELL_SECONDS;
        setFeeding(feed);
        if (feed && !shooting) {
            enter(collected ? Phase.SHOOT_COLLECTED : Phase.SHOOT_PRELOAD,
                    now, collected ? "Timed garden batch" : "Timed four-pollen preload batch");
        }
    }

    private void checkHiveAfterCollection(double now) {
        if (now - phaseStartedAt < HIVE_SETTLE_SECONDS) return;
        VisionSubsystem.DetectionSnapshot snapshot = vision.update(configuration.getAlliance());
        ClusterDetection cell = snapshot.hasValidResult
                ? HiveTrackingController.findNearestTrackableCluster(snapshot.clusters, true) : null;
        if (cell == null || cell.alliance != configuration.getAlliance()) {
            hiveStatus = "No visible scorable alliance hive after collection";
            park(hiveStatus, now);
            return;
        }
        hiveSide = cell.audienceSide ? StartingSpot.AUDIENCE : StartingSpot.SCORING;
        hiveStatus = cell.name + " confirmed scorable after collection";
        driveToHive(true, now);
    }

    private ClusterDetection findCell(VisionSubsystem.DetectionSnapshot snapshot) {
        if (!snapshot.hasValidResult) return null;
        for (ClusterDetection cell : snapshot.clusters) {
            if (cell.alliance == configuration.getAlliance()
                    && cell.audienceSide == (hiveSide == StartingSpot.AUDIENCE)) return cell;
        }
        return null;
    }

    private void driveToHive(boolean collected, double now) {
        stopMechanisms();
        alignedSince = Double.NaN;
        aim.reset();
        startNavigation(map.hiveShootingDestination(configuration.getAlliance(), hiveSide),
                collected ? Phase.DRIVE_COLLECTED : Phase.DRIVE_PRELOAD, now);
    }

    private void startNavigation(LandmarkFieldMap.Destination target, Phase next, double now) {
        if (!navigation.start(target, now)) {
            halt(navigation.getStatus());
            return;
        }
        enter(next, now, "Travelling to " + target.name);
    }

    private void enter(Phase next, double now, String message) {
        phase = next;
        phaseStartedAt = now;
        status = message;
    }

    private void park(String reason, double now) {
        stopMechanisms();
        stopDrive();
        aim.reset();
        parkingReason = reason;
        startNavigation(map.parkingDestination(configuration.getAlliance(),
                configuration.getStartingSpot()), Phase.DRIVE_PARK, now);
        if (phase == Phase.DRIVE_PARK) status = "Parking: " + reason;
    }

    private void setFeeding(boolean enabled) {
        feeding = enabled;
        feeder.update(enabled);
        intake.setPower(enabled ? 0.5 : 0);
    }

    private void stopMechanisms() {
        feeding = false;
        if (feederReady) feeder.update(false);
        if (intakeReady) intake.setPower(0);
        if (launcherReady) launcher.update(false);
    }

    private void stopDrive() {
        if (driveReady) drive.driveRobotRelative(0, 0, 0);
    }

    private void halt(String message) {
        phase = Phase.STOPPED;
        status = message;
        if (navigation != null) navigation.cancel(message);
        if (aim != null) aim.reset();
        stopMechanisms();
        stopDrive();
    }

    @Override public void stop() {
        halt("OpMode stopped");
        vision.stop();
    }

    private void publish() {
        telemetry.clearAll();
        if (!started) configuration.publish();
        telemetry.addLine("Auto Shoot");
        telemetry.addData("Phase", phase);
        telemetry.addLine(status);
        telemetry.addData("Hive", hiveStatus);
        telemetry.addData("Batch feed time (not ball count)", "%.2f / %.2f s",
                feedSeconds, BATCH_FEED_SECONDS);
        telemetry.addLine("Timed feeding/collection: verify four-pollen timing and launcher speed");
        if (profile != null && !profile.isAutonomousReady()) {
            telemetry.addLine("Autonomous blocked: profile geometry/tuning not verified or simulator disabled");
        }
        telemetry.update();
    }
}
