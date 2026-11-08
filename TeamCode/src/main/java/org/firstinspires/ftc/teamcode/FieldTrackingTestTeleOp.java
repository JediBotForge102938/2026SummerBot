package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.BioBuzzLimelightClusterAdapter.Alliance;
import org.firstinspires.ftc.teamcode.control.DriveModePreferences;
import org.firstinspires.ftc.teamcode.control.ApproximateVisionPosition;
import org.firstinspires.ftc.teamcode.subsystems.ConfigurationSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FieldMap;
import org.firstinspires.ftc.teamcode.subsystems.RobotProfile;
import org.firstinspires.ftc.teamcode.subsystems.TrackingSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.telemetry.TelemetryTable;
import java.util.Locale;

@TeleOp(name = "Field Tracking Test", group = "2 - (Tests)")
public class FieldTrackingTestTeleOp extends OpMode {
    private final DriveSubsystem drive = new DriveSubsystem();
    private final ConfigurationSubsystem configuration = new ConfigurationSubsystem();
    private final TrackingSubsystem tracking = new TrackingSubsystem();
    private final VisionSubsystem vision = new VisionSubsystem();
    private RobotProfile profile;
    private boolean driveReady;

    @Override public void init() {
        profile = RobotProfile.loadActive(null);
        drive.init(hardwareMap, profile);
        driveReady = true;
        drive.driveRobotRelative(0, 0, 0);
        tracking.init(hardwareMap, profile);
        vision.init(hardwareMap, profile);
        configuration.init(telemetry, gamepad1, new DriveModePreferences(hardwareMap.appContext));
        configuration.setRobotProfileName(profile.name);
        publish(null, Double.NaN);
    }

    @Override public void init_loop() {
        configuration.updateInit(gamepad1);
        publish(null, Double.NaN);
    }

    @Override public void start() {
        configuration.lock();
        FieldMap.Pose start = FieldMap.startingPose(configuration.getAlliance(),
                configuration.getStartingSpot(), profile.chassisLengthCm / 2.54);
        drive.resetHeading(start.headingDegrees, configuration.getFieldForwardHeadingDegrees());
        tracking.start(start);
    }

    @Override public void loop() {
        double heading = drive.getHeadingDegrees();
        tracking.update(heading);
        if (gamepad1.y || !Double.isFinite(heading)) {
            drive.driveRobotRelative(0, 0, 0);
        } else if (configuration.isFieldCentric()) {
            drive.driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x,
                    gamepad1.right_stick_x, 0.25);
        } else {
            drive.driveRobotRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x,
                    gamepad1.right_stick_x, 0.25);
        }
        publish(vision.update(Alliance.BOTH), heading);
    }

    private void publish(VisionSubsystem.DetectionSnapshot visionResult, double heading) {
        telemetry.clearAll();
        configuration.publish();
        if (visionResult != null) {
            telemetry.addLine("");
            TelemetryTable.heading(telemetry, "Field tracking");
            TelemetryTable.row(telemetry, "Status", tracking.getStatus());
            FieldMap.Pose pose = tracking.getPose();
            TelemetryTable.row(telemetry, "Robot pose", pose == null ? "Unavailable"
                    : String.format(Locale.US, "X %.1f Y %.1f H %.1f", pose.x, pose.y, pose.headingDegrees));
            TelemetryTable.row(telemetry, "OTOS heading", String.format(Locale.US, "%.1f", tracking.getOtosHeading()));
            telemetry.addLine("");
            TelemetryTable.heading(telemetry, "Vision (approximate; not fused)");
            TelemetryTable.row(telemetry, "Status", visionResult.hasValidResult
                    ? visionResult.clusters.size() + " cell(s)" : "Searching; not fused");
            if (visionResult.hasValidResult) {
                TelemetryTable.row(telemetry, "Age", String.format(Locale.US, "%.0f ms", visionResult.observationAgeMs));
            }
            for (BioBuzzLimelightClusterAdapter.ClusterDetection cell : visionResult.clusters) {
                TelemetryTable.heading(telemetry, cell.name);
                TelemetryTable.row(telemetry, "Cell status", VisionSubsystem.classifyCell(cell).toString());
                if (cell.hasPose()) {
                    TelemetryTable.row(telemetry, "Camera XYZ", String.format(Locale.US, "%.1f %.1f %.1f in",
                            cell.poseCameraSpace.getPosition().x, cell.poseCameraSpace.getPosition().y,
                            cell.poseCameraSpace.getPosition().z));
                }
                FieldMap.Pose estimate = ApproximateVisionPosition.estimate(cell, profile, heading);
                if (estimate == null) {
                    TelemetryTable.row(telemetry, "Vision XY", "Unavailable");
                } else {
                    TelemetryTable.row(telemetry, "Vision XY", String.format(Locale.US,
                            "%.1f %.1f in (approx)", estimate.x, estimate.y));
                    TelemetryTable.row(telemetry, "Vision - OTOS", pose == null ? "Unavailable"
                            : String.format(Locale.US, "dX %+.1f dY %+.1f in",
                                    estimate.x - pose.x, estimate.y - pose.y));
                }
            }
        }
        telemetry.update();
    }

    @Override public void stop() {
        if (driveReady) drive.driveRobotRelative(0, 0, 0);
        vision.stop();
    }
}
