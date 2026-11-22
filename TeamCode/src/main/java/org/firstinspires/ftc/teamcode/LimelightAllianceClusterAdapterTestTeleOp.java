package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotProfile;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Limelight Alliance Cluster Adapter Test", group = "2 - (Tests)")
public class LimelightAllianceClusterAdapterTestTeleOp extends OpMode {
    private final VisionSubsystem vision = new VisionSubsystem();
    private RobotProfile profile;
    private BioBuzzLimelightClusterAdapter.Alliance selectedAlliance =
            BioBuzzLimelightClusterAdapter.Alliance.BLUE;

    @Override
    public void init() {
        profile = RobotProfile.loadActive(null);
        vision.init(hardwareMap, profile);
        telemetry.addData("Robot profile", profile.name);
        telemetry.addData("Limelight", profile.limelightName);
        telemetry.addData("Pipeline", profile.limelightPipeline);
        telemetry.addData("Camera mounting", "pitch up %.1f deg, lens roll %.1f deg, inverted %s",
                profile.limelightPitchUpDegrees, profile.limelightRollDegrees, profile.limelightInverted);
        telemetry.addData("Selected alliance", selectedAlliance);
        telemetry.addLine("Press Options to switch alliance");
    }

    @Override public void init_loop() {
        telemetry.addData("Robot profile", profile.name);
        telemetry.addData("Camera pitch up", "%.1f deg", profile.limelightPitchUpDegrees);
        telemetry.addData("Selected alliance", selectedAlliance);
        telemetry.addLine("Press Options to switch alliance");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.optionsWasPressed()) {
            selectedAlliance = selectedAlliance == BioBuzzLimelightClusterAdapter.Alliance.BLUE
                    ? BioBuzzLimelightClusterAdapter.Alliance.RED
                    : BioBuzzLimelightClusterAdapter.Alliance.BLUE;
        }

        telemetry.addData("Selected alliance", selectedAlliance);
        telemetry.addLine("Press Options to switch alliance");
        telemetry.addData("Robot profile", profile.name);
        telemetry.addData("Camera pitch up", "%.1f deg", profile.limelightPitchUpDegrees);
        telemetry.addLine("Opening-up > 0: scorable; < 0: down; horizontal/disagreement: unknown");
        telemetry.addLine("Mount compensation assumes robot is level");

        VisionSubsystem.DetectionSnapshot snapshot = vision.update(selectedAlliance);
        if (!snapshot.hasValidResult) {
            telemetry.addLine("No valid Limelight result");
            telemetry.update();
            return;
        }

        List<String> downCells = new ArrayList<>();
        List<String> unknownPoseCells = new ArrayList<>();
        if (snapshot.clusters.isEmpty()) {
            telemetry.addLine("No selected-alliance BioBuzz cluster tags detected");
        }
        for (BioBuzzLimelightClusterAdapter.ClusterDetection cluster : snapshot.clusters) {
            telemetry.addLine("==== " + cluster.name);
            telemetry.addData(
                    "Tags found",
                    "%d/4 (%d%%): %s",
                    cluster.visibleTagIds.size(),
                    cluster.percentClusterFound,
                    cluster.visibleTagIds);
            telemetry.addData("Tags left to right", cluster.visibleTagIdsLeftToRight);

            if (!cluster.hasPose()) {
                telemetry.addLine("Cell status: UNKNOWN (pose unavailable)");
                unknownPoseCells.add(cluster.name);
                continue;
            }

            Pose3D pose = cluster.poseCameraSpace;
            telemetry.addData(
                    "Opening origin (in)",
                    "X %6.2f right, Y %6.2f down, Z %6.2f forward",
                    pose.getPosition().x,
                    pose.getPosition().y,
                    pose.getPosition().z);
            telemetry.addData(
                    "Camera-space yaw / pitch / roll (deg)",
                    "%6.1f / %6.1f / %6.1f",
                    pose.getOrientation().getYaw(AngleUnit.DEGREES),
                    pose.getOrientation().getPitch(AngleUnit.DEGREES),
                    pose.getOrientation().getRoll(AngleUnit.DEGREES));

            double clusterRoll = VisionSubsystem.getClusterRollDegrees(cluster);
            VisionSubsystem.CellStatus cellStatus = VisionSubsystem.classifyCell(cluster);
            telemetry.addData(
                    "Cell status",
                    "%s (ascending IDs and all opening normals up)",
                    cellStatus);
            telemetry.addData("Mount-compensated roll", "%.1f deg", clusterRoll);
            telemetry.addData("Opening normal up (verify sign)", "%.3f; tag range %.3f to %.3f",
                    cluster.openingNormalUp, cluster.minOpeningNormalUp, cluster.maxOpeningNormalUp);
            telemetry.addLine("Known raised cell: expect positive opening-up; assumes tag +Z outward");
            if (cellStatus == VisionSubsystem.CellStatus.UNKNOWN_POSE) {
                unknownPoseCells.add(cluster.name);
            } else if (cellStatus == VisionSubsystem.CellStatus.NOT_SCORABLE) {
                downCells.add(cluster.name);
            }
        }

        telemetry.addData(
                "Down / not scorable cells",
                downCells.isEmpty() ? "none" : String.join(", ", downCells));
        telemetry.addData(
                "Unknown-pose cells",
                unknownPoseCells.isEmpty() ? "none" : String.join(", ", unknownPoseCells));
        telemetry.update();
    }

    @Override
    public void stop() {
        vision.stop();
    }
}
