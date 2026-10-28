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

@TeleOp(name = "BioBuzz AprilTag Test", group = "Vision")
public class BioBuzzAprilTagTest extends OpMode {
    private static final String LIMELIGHT_NAME = "limelight";
    // This slot must be configured as an AprilTag pipeline in the Limelight web UI.
    private static final int LIMELIGHT_PIPELINE = 0;

    private static final String[] RED_CLUSTER_NAMES = {
            "RED SCORING",
            "RED AUDIENCE"
    };

    private static final int[][] RED_CLUSTER_TAG_IDS = {
            {30, 31, 32, 33},
            {34, 35, 36, 37}
    };

    private Limelight3A limelight;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
        limelight.start();

        telemetry.addData("Limelight", LIMELIGHT_NAME);
        telemetry.addData("Pipeline", LIMELIGHT_PIPELINE);
        telemetry.addData("Cluster filter", "Red");
        telemetry.addData("Status", "Ready");
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();
        Map<Integer, LLResultTypes.FiducialResult> detectedRedTags = new HashMap<>();

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                int id = fiducial.getFiducialId();
                if (isRedClusterTag(id)) {
                    detectedRedTags.put(id, fiducial);
                }
            }
        } else {
            telemetry.addLine("No valid Limelight result");
        }

        for (int clusterIndex = 0; clusterIndex < RED_CLUSTER_NAMES.length; clusterIndex++) {
            int[] tagIds = RED_CLUSTER_TAG_IDS[clusterIndex];
            StringBuilder visibleIds = new StringBuilder();

            for (int id : tagIds) {
                if (detectedRedTags.containsKey(id)) {
                    if (visibleIds.length() > 0) {
                        visibleIds.append(", ");
                    }
                    visibleIds.append(id);
                }
            }

            int visibleCount = detectedRedTagsFor(tagIds, detectedRedTags);
            telemetry.addLine("==== " + RED_CLUSTER_NAMES[clusterIndex]);
            telemetry.addData("Tags found", "%d/%d (%d%%): %s",
                    visibleCount, tagIds.length, visibleCount * 100 / tagIds.length,
                    visibleIds.length() == 0 ? "none" : visibleIds.toString());

            int poseCount = 0;
            int uprightCount = 0;
            for (int id : tagIds) {
                LLResultTypes.FiducialResult fiducial = detectedRedTags.get(id);
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

    private boolean isRedClusterTag(int id) {
        for (int[] tagIds : RED_CLUSTER_TAG_IDS) {
            for (int tagId : tagIds) {
                if (id == tagId) {
                    return true;
                }
            }
        }
        return false;
    }

    private int detectedRedTagsFor(
            int[] tagIds,
            Map<Integer, LLResultTypes.FiducialResult> detectedRedTags) {
        int count = 0;
        for (int id : tagIds) {
            if (detectedRedTags.containsKey(id)) {
                count++;
            }
        }
        return count;
    }
}
