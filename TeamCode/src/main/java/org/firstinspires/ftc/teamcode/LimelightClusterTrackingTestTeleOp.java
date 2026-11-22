package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.control.HiveTrackingController;
import org.firstinspires.ftc.teamcode.control.ImuAimController;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotProfile;

@TeleOp(name = "Limelight Cluster Tracking Test", group = "2 - (Tests)")
public class LimelightClusterTrackingTestTeleOp extends OpMode {
    private static final double FOLLOW_BACKWARD_POWER = -0.25;

    private final DriveSubsystem drive = new DriveSubsystem();
    private final VisionSubsystem vision = new VisionSubsystem();
    private final ImuAimController aim = new ImuAimController(drive::getYawRateDegreesPerSecond);
    private BioBuzzLimelightClusterAdapter.Alliance selectedAlliance =
            BioBuzzLimelightClusterAdapter.Alliance.BLUE;
    private boolean driveInitialized;
    private RobotProfile profile;

    @Override
    public void init() {
        profile = RobotProfile.loadActive(null);
        drive.init(hardwareMap, profile);
        driveInitialized = true;
        vision.init(hardwareMap, profile);
        telemetry.addData("Robot profile", profile.name);
        telemetry.addData("Selected alliance", selectedAlliance);
        telemetry.addLine("OPTIONS: switch alliance");
        telemetry.addLine("Hold A: turn to face nearest cluster");
        telemetry.addLine("Hold B: turn and drive toward nearest cluster");
        telemetry.addData("Limelight", profile.limelightName);
        telemetry.addData("Pipeline", profile.limelightPipeline);
        telemetry.addData("Camera mounting", "pitch up %.1f deg, lens roll %.1f deg, inverted %s",
                profile.limelightPitchUpDegrees, profile.limelightRollDegrees, profile.limelightInverted);
    }

    @Override public void init_loop() {
        telemetry.addData("Robot profile", profile.name);
        telemetry.addData("Camera pitch up", "%.1f deg", profile.limelightPitchUpDegrees);
        telemetry.addData("Selected alliance", selectedAlliance);
        telemetry.addLine("OPTIONS: switch alliance | Hold A: track | Hold B: follow");
        telemetry.update();
    }

    @Override
    public void loop() {
        aim.sample();
        if (gamepad1.optionsWasPressed()) {
            selectedAlliance = selectedAlliance == BioBuzzLimelightClusterAdapter.Alliance.BLUE
                    ? BioBuzzLimelightClusterAdapter.Alliance.RED
                    : BioBuzzLimelightClusterAdapter.Alliance.BLUE;
        }

        VisionSubsystem.DetectionSnapshot snapshot = vision.update(selectedAlliance);
        BioBuzzLimelightClusterAdapter.ClusterDetection target =
                HiveTrackingController.findNearestTrackableCluster(snapshot.clusters, false);

        double forward = 0.0;
        double rotate = 0.0;
        String mode = "Idle";
        if (gamepad1.b) {
            mode = "Follow";
        } else if (gamepad1.a) {
            mode = "Track";
        }

        if (target == null) {
            drive.driveRobotRelative(0.0, 0.0, 0.0);
            telemetry.addData(
                    "Tracking",
                    snapshot.hasValidResult
                            ? "No selected-alliance cluster with a forward pose"
                            : "No valid Limelight result");
        } else {
            Pose3D pose = target.poseCameraSpace;
            double targetX = pose.getPosition().x;
            double targetZ = pose.getPosition().z;
            double horizontalAngleDegrees = Math.toDegrees(Math.atan2(targetX, targetZ));
            if (gamepad1.a || gamepad1.b) {
                // This rear-facing camera needs the opposite chassis turn from a front-facing camera.
                rotate = aim.turn(horizontalAngleDegrees, snapshot.observationAgeMs);
                telemetry.addData("IMU corrected error", "%.1f deg", aim.getCorrectedAngle());
                telemetry.addData("Motion compensation", aim.isCompensated()
                        ? "IMU rotation" : "Warming up (vision + rate damping)");
            }
            if (gamepad1.b) {
                forward = FOLLOW_BACKWARD_POWER;
            }

            drive.driveRobotRelative(forward, 0.0, rotate);
            telemetry.addData("Target", target.name);
            telemetry.addData("Visible tags", target.visibleTagIds);
            telemetry.addData("Tags left to right", target.visibleTagIdsLeftToRight);
            telemetry.addData("Target range", "%.1f in", Math.hypot(targetX, targetZ));
            telemetry.addData("Horizontal error", "%.1f deg", horizontalAngleDegrees);
            telemetry.addData("Turn power", "%.2f", rotate);
            telemetry.addData(
                    "Cell status",
                    "%s (ascending IDs and all opening normals up)",
                    VisionSubsystem.classifyCell(target));
            telemetry.addData(
                    "Mount-compensated roll",
                    "%.1f deg",
                    VisionSubsystem.getClusterRollDegrees(target));
            telemetry.addData("Camera-space roll", "%.1f deg",
                    VisionSubsystem.getCameraSpaceRollDegrees(target));
            telemetry.addData("Opening normal up (verify sign)", "%.3f; tag range %.3f to %.3f",
                    target.openingNormalUp, target.minOpeningNormalUp, target.maxOpeningNormalUp);
            telemetry.addLine("Known raised cell: expect positive opening-up; assumes tag +Z outward");
            telemetry.addLine("Opening-up > 0: scorable; < 0: down; horizontal/disagreement: unknown");
        }

        telemetry.addData("Selected alliance", selectedAlliance);
        telemetry.addData("Robot profile", profile.name);
        telemetry.addData("Camera pitch up", "%.1f deg", profile.limelightPitchUpDegrees);
        telemetry.addData("Mode", mode);
        telemetry.addLine("OPTIONS: switch alliance | Hold A: track | Hold B: follow");
        telemetry.update();
    }

    @Override
    public void stop() {
        aim.reset();
        if (driveInitialized) {
            drive.driveRobotRelative(0.0, 0.0, 0.0);
        }
        vision.stop();
    }
}
