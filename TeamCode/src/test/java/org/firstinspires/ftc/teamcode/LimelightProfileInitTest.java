package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.io.File;
import java.util.Arrays;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotProfile;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.junit.Test;
import org.mockito.MockedConstruction;
import org.mockito.MockedStatic;

public class LimelightProfileInitTest {
    @Test public void clusterTestsShowBothRollReferencesDuringTracking() {
        RobotProfile profile = RobotProfile.load(new File("../robot-configs"),
                "biobuzz-competitionbot", "competitionbot");
        HardwareMap hardware = mock(HardwareMap.class);
        Limelight3A camera = mock(Limelight3A.class);
        LLResult result = mock(LLResult.class);
        when(hardware.get(Limelight3A.class, profile.limelightName)).thenReturn(camera);
        when(camera.getLatestResult()).thenReturn(result);
        when(result.isValid()).thenReturn(true);
        LLResultTypes.FiducialResult first = mock(LLResultTypes.FiducialResult.class);
        LLResultTypes.FiducialResult second = mock(LLResultTypes.FiducialResult.class);
        when(first.getFiducialId()).thenReturn(38);
        when(second.getFiducialId()).thenReturn(39);
        when(first.getTargetPoseCameraSpace()).thenReturn(new Pose3D(
                new Position(DistanceUnit.INCH, 20, 0, 100, 0),
                new YawPitchRollAngles(AngleUnit.DEGREES, 180, 0,
                        -10 - profile.limelightPitchUpDegrees, 0)));
        when(second.getTargetPoseCameraSpace()).thenReturn(new Pose3D(
                new Position(DistanceUnit.INCH, 10, 0, 100, 0),
                new YawPitchRollAngles(AngleUnit.DEGREES, 180, 0,
                        -10 - profile.limelightPitchUpDegrees, 0)));
        when(result.getFiducialResults()).thenReturn(Arrays.asList(first, second));
        VisionSubsystem realVision = new VisionSubsystem();
        realVision.init(hardware, profile);
        VisionSubsystem.DetectionSnapshot snapshot = realVision.update(
                BioBuzzLimelightClusterAdapter.Alliance.BLUE);
        realVision.stop();
        for (int mode = 0; mode < 2; mode++) {
            try (MockedStatic<RobotProfile> profiles = mockStatic(RobotProfile.class);
                 MockedConstruction<DriveSubsystem> drives = mockConstruction(DriveSubsystem.class);
                 MockedConstruction<VisionSubsystem> visions = mockConstruction(VisionSubsystem.class,
                         (vision, context) -> when(vision.update(any())).thenReturn(snapshot))) {
                profiles.when(() -> RobotProfile.loadActive(null)).thenReturn(profile);
                OpMode op = mode == 0 ? new LimelightClusterTrackingTestTeleOp()
                        : new LimelightAllianceClusterAdapterTestTeleOp();
                op.hardwareMap = hardware;
                op.telemetry = mock(Telemetry.class);
                op.gamepad1 = mock(Gamepad.class);
                op.init();
                clearInvocations(op.telemetry);
                op.loop();
                verify(op.telemetry).addData("Robot profile", profile.name);
                verify(op.telemetry).addData("Camera pitch up", "%.1f deg", 37.0);
                verify(op.telemetry).addData(eq("Mount-compensated roll"), eq("%.1f deg"),
                        doubleThat(value -> Math.abs(value + 10) < 1e-8));
                verify(op.telemetry).addData("Cell status", "%s (roll < %.1f deg)",
                        VisionSubsystem.CellStatus.SCORABLE, 65.0);
                if (mode == 0) {
                    verify(op.telemetry).addData(eq("Tag 38 roll"),
                            eq("camera %.1f / level %.1f deg"),
                            doubleThat(value -> Math.abs(value + 10
                                    + profile.limelightPitchUpDegrees) < 1e-8),
                            doubleThat(value -> Math.abs(value + 10) < 1e-8));
                    verify(op.telemetry).addData(eq("Tag 39 roll"),
                            eq("camera %.1f / level %.1f deg"),
                            doubleThat(value -> Math.abs(value + 10
                                    + profile.limelightPitchUpDegrees) < 1e-8),
                            doubleThat(value -> Math.abs(value + 10) < 1e-8));
                } else {
                    verify(op.telemetry).addData("Opening normal up (verify sign)",
                            "%.3f; tag range %.3f to %.3f",
                            snapshot.clusters.get(0).openingNormalUp,
                            snapshot.clusters.get(0).minOpeningNormalUp,
                            snapshot.clusters.get(0).maxOpeningNormalUp);
                    verify(op.telemetry).addData(eq("Camera-space yaw / pitch / roll (deg)"),
                            eq("%6.1f / %6.1f / %6.1f"),
                            doubleThat(value -> Math.abs(value) < 1e-8), eq(0.0),
                            doubleThat(value -> Math.abs(value + 10
                                    + profile.limelightPitchUpDegrees) < 1e-8));
                }
                op.stop();
            }
        }
    }

    @Test public void allLimelightTestsUseActiveProfileAndShowItThroughoutInit() {
        for (String robot : new String[] {"starterbot", "competitionbot"}) {
            RobotProfile profile = RobotProfile.load(new File("../robot-configs"),
                    "biobuzz-" + robot, robot);
            for (int mode = 0; mode < 2; mode++) {
                try (MockedStatic<RobotProfile> profiles = mockStatic(RobotProfile.class);
                     MockedConstruction<DriveSubsystem> drives = mockConstruction(DriveSubsystem.class);
                     MockedConstruction<VisionSubsystem> visions = mockConstruction(VisionSubsystem.class)) {
                    profiles.when(() -> RobotProfile.loadActive(null)).thenReturn(profile);
                    OpMode op = mode == 0 ? new LimelightClusterTrackingTestTeleOp()
                            : new LimelightAllianceClusterAdapterTestTeleOp();
                    op.hardwareMap = mock(HardwareMap.class);
                    op.telemetry = mock(Telemetry.class);
                    op.gamepad1 = mock(Gamepad.class);
                    op.init();
                    profiles.verify(() -> RobotProfile.loadActive(null));
                    verify(op.telemetry).addData("Robot profile", profile.name);
                    verify(visions.constructed().get(0)).init(op.hardwareMap, profile);
                    verify(op.telemetry).addData("Camera mounting",
                            "pitch up %.1f deg, lens roll %.1f deg, inverted %s",
                            profile.limelightPitchUpDegrees, profile.limelightRollDegrees,
                            profile.limelightInverted);
                    if (mode == 0) {
                        verify(drives.constructed().get(0)).init(op.hardwareMap, profile);
                    }
                    clearInvocations(op.telemetry);
                    op.init_loop();
                    verify(op.telemetry).addData("Robot profile", profile.name);
                    verify(op.telemetry).addData("Camera pitch up", "%.1f deg",
                            profile.limelightPitchUpDegrees);
                    op.stop();
                }
            }
        }
    }
}
