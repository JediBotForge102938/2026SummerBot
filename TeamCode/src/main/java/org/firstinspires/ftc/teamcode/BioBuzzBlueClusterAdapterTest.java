package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "BioBuzz Blue Cluster Adapter Test", group = "Vision")
public class BioBuzzBlueClusterAdapterTest extends OpMode {
    private final VisionSubsystem vision = new VisionSubsystem();

    @Override
    public void init() {
        vision.init(hardwareMap);
        telemetry.addData("Limelight", VisionSubsystem.LIMELIGHT_NAME);
        telemetry.addData("Pipeline", VisionSubsystem.LIMELIGHT_PIPELINE);
        telemetry.addData("Cluster filter", "Blue");
        telemetry.addData("Status", "Ready");
    }

    @Override
    public void loop() {
        telemetry.addLine("Roll check assumes camera optical axis aligns with launcher");
        VisionSubsystem.DetectionSnapshot snapshot =
                vision.update(BioBuzzLimelightClusterAdapter.Alliance.BLUE);
        if (!snapshot.hasValidResult) {
            telemetry.addLine("No valid Limelight result");
            telemetry.update();
            return;
        }

        List<String> downCells = new ArrayList<>();
        List<String> unknownPoseCells = new ArrayList<>();
        if (snapshot.clusters.isEmpty()) {
            telemetry.addLine("No blue BioBuzz cluster tags detected");
        }
        for (BioBuzzLimelightClusterAdapter.ClusterDetection cluster : snapshot.clusters) {
            telemetry.addLine("==== " + cluster.name);
            telemetry.addData(
                    "Tags found",
                    "%d/4 (%d%%): %s",
                    cluster.visibleTagIds.size(),
                    cluster.percentClusterFound,
                    cluster.visibleTagIds);

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
                    "Yaw / pitch / roll (deg)",
                    "%6.1f / %6.1f / %6.1f",
                    pose.getOrientation().getYaw(AngleUnit.DEGREES),
                    pose.getOrientation().getPitch(AngleUnit.DEGREES),
                    pose.getOrientation().getRoll(AngleUnit.DEGREES));

            double ftcRoll = VisionSubsystem.getFtcRollDegrees(cluster);
            VisionSubsystem.CellStatus cellStatus = VisionSubsystem.classifyCell(cluster);
            telemetry.addData(
                    "Cell status",
                    "%s (FTC-equivalent roll %.1f deg)",
                    cellStatus,
                    ftcRoll);
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
