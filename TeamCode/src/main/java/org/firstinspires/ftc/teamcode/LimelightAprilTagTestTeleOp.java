package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

@TeleOp(name = "Limelight AprilTag Test", group = "2 - (Tests)")
public class LimelightAprilTagTestTeleOp extends OpMode {
    private static final String LIMELIGHT_NAME = "limelight";
    // This slot must be configured as an AprilTag pipeline in the Limelight web UI.
    private static final int LIMELIGHT_PIPELINE = 0;

    private enum Alliance {
        BLUE,
        RED;

        private Alliance opposite() {
            return this == BLUE ? RED : BLUE;
        }
    }

    private static final String[] BLUE_CLUSTER_NAMES = {
            "BLUE SCORING",
            "BLUE AUDIENCE"
    };

    private static final int[][] BLUE_CLUSTER_TAG_IDS = {
            {42, 43, 44, 45},
            {38, 39, 40, 41}
    };

    private static final String[] RED_CLUSTER_NAMES = {
            "RED SCORING",
            "RED AUDIENCE"
    };

    private static final int[][] RED_CLUSTER_TAG_IDS = {
            {30, 31, 32, 33},
            {34, 35, 36, 37}
    };

    private Limelight3A limelight;
    private Alliance selectedAlliance = Alliance.BLUE;

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
            selectedAlliance = selectedAlliance.opposite();
        }

        telemetry.addData("Selected Team", selectedAlliance);
        telemetry.addData("Controls", "Press OPTIONS to switch alliances");

        LLResult result = limelight.getLatestResult();
        Map<Integer, LLResultTypes.FiducialResult> detectedAllianceTags = new HashMap<>();

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                int id = fiducial.getFiducialId();
                if (isSelectedAllianceTag(id)) {
                    detectedAllianceTags.put(id, fiducial);
                }
            }
        } else {
            telemetry.addLine("No valid Limelight result");
        }

        String[] clusterNames = selectedAlliance == Alliance.BLUE
                ? BLUE_CLUSTER_NAMES : RED_CLUSTER_NAMES;
        int[][] clusterTagIds = selectedAlliance == Alliance.BLUE
                ? BLUE_CLUSTER_TAG_IDS : RED_CLUSTER_TAG_IDS;
        for (int clusterIndex = 0; clusterIndex < clusterNames.length; clusterIndex++) {
            int[] tagIds = clusterTagIds[clusterIndex];
            StringBuilder visibleIds = new StringBuilder();

            for (int id : tagIds) {
                if (detectedAllianceTags.containsKey(id)) {
                    if (visibleIds.length() > 0) {
                        visibleIds.append(", ");
                    }
                    visibleIds.append(id);
                }
            }

            int visibleCount = detectedAllianceTagsFor(tagIds, detectedAllianceTags);
            telemetry.addLine("==== " + clusterNames[clusterIndex]);
            telemetry.addData("Tags found", "%d/%d (%d%%): %s",
                    visibleCount, tagIds.length, visibleCount * 100 / tagIds.length,
                    visibleIds.length() == 0 ? "none" : visibleIds.toString());

            int poseCount = 0;
            int uprightCount = 0;
            for (int id : tagIds) {
                LLResultTypes.FiducialResult fiducial = detectedAllianceTags.get(id);
                if (fiducial != null) {
                    Pose3D tagPose = fiducial.getTargetPoseCameraSpace();
                    if (tagPose == null) {
                        telemetry.addData("ID " + id, "pose unavailable");
                    } else {
                        double roll = tagPose.getOrientation().getRoll(AngleUnit.DEGREES);
                        boolean upright = Math.abs(roll) < 90.0;
                        poseCount++;
                        if (upright) {
                            uprightCount++;
                        }
                        telemetry.addData("ID " + id,
                                "tx %.1f, ty %.1f, roll %.1f deg (%s)",
                                fiducial.getTargetXDegrees(),
                                fiducial.getTargetYDegrees(),
                                roll,
                                upright ? "upright" : "upside down");
                    }
                }
            }

            String orientation;
            if (poseCount == 0) {
                orientation = "unknown";
            } else if (uprightCount == poseCount) {
                orientation = "upright";
            } else if (uprightCount == 0) {
                orientation = "upside down";
            } else {
                orientation = "mixed tag poses";
            }
            telemetry.addData("Cell orientation", orientation);
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
    }

    private boolean isSelectedAllianceTag(int id) {
        int[][] clusterTagIds = selectedAlliance == Alliance.BLUE
                ? BLUE_CLUSTER_TAG_IDS : RED_CLUSTER_TAG_IDS;
        for (int[] tagIds : clusterTagIds) {
            for (int tagId : tagIds) {
                if (id == tagId) {
                    return true;
                }
            }
        }
        return false;
    }

    private int detectedAllianceTagsFor(
            int[] tagIds,
            Map<Integer, LLResultTypes.FiducialResult> detectedAllianceTags) {
        int count = 0;
        for (int id : tagIds) {
            if (detectedAllianceTags.containsKey(id)) {
                count++;
            }
        }
        return count;
    }
}
