package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp(name = "BioBuzz Alliance Cluster Adapter Test", group = "Vision")
public class BioBuzzAllianceClusterAdapterTest extends OpMode {
    private static final String LIMELIGHT_NAME = "limelight";
    // Configure this slot for 36h11 AprilTags with a 3.25-inch tag size.
    private static final int LIMELIGHT_PIPELINE = 0;

    private Limelight3A limelight;
    private final BioBuzzLimelightClusterAdapter clusterAdapter =
            new BioBuzzLimelightClusterAdapter();
    private BioBuzzLimelightClusterAdapter.Alliance selectedAlliance =
            BioBuzzLimelightClusterAdapter.Alliance.BLUE;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
        limelight.start();

        telemetry.addData("Selected Team", selectedAlliance);
        telemetry.addData("Controls", "Press OPTIONS to switch alliances");
        telemetry.addData("Limelight", LIMELIGHT_NAME);
        telemetry.addData("Pipeline", LIMELIGHT_PIPELINE);
        telemetry.addData("Status", "Ready");
    }

    @Override
    public void loop() {
        if (gamepad1.optionsWasPressed()) {
            selectedAlliance = selectedAlliance == BioBuzzLimelightClusterAdapter.Alliance.BLUE
                    ? BioBuzzLimelightClusterAdapter.Alliance.RED
                    : BioBuzzLimelightClusterAdapter.Alliance.BLUE;
        }

        telemetry.addData("Selected Team", selectedAlliance);
        telemetry.addData("Controls", "Press OPTIONS to switch alliances");

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            telemetry.addLine("No valid Limelight result");
            telemetry.update();
            return;
        }

        List<BioBuzzLimelightClusterAdapter.ClusterDetection> clusters =
                clusterAdapter.detect(result, selectedAlliance);
        if (clusters.isEmpty()) {
            telemetry.addLine("No " + selectedAlliance + " BioBuzz cluster tags detected");
        }

        for (BioBuzzLimelightClusterAdapter.ClusterDetection cluster : clusters) {
            telemetry.addLine("==== " + cluster.name);
            telemetry.addData(
                    "Tags found",
                    "%d/4 (%d%%): %s",
                    cluster.visibleTagIds.size(),
                    cluster.percentClusterFound,
                    cluster.visibleTagIds);

            if (!cluster.hasPose()) {
                telemetry.addLine("Cluster pose unavailable");
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
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }
}
