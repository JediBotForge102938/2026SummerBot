package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.io.File;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotProfile;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.junit.Test;
import org.mockito.MockedConstruction;
import org.mockito.MockedStatic;

public class LimelightProfileInitTest {
    @Test public void allLimelightTestsUseActiveProfileAndShowItThroughoutInit() {
        for (String robot : new String[] {"starterbot", "competitionbot"}) {
            RobotProfile profile = RobotProfile.load(new File("../robot-configs"),
                    "biobuzz-" + robot, robot);
            for (int mode = 0; mode < 3; mode++) {
                try (MockedStatic<RobotProfile> profiles = mockStatic(RobotProfile.class);
                     MockedConstruction<DriveSubsystem> drives = mockConstruction(DriveSubsystem.class);
                     MockedConstruction<VisionSubsystem> visions = mockConstruction(VisionSubsystem.class)) {
                    profiles.when(() -> RobotProfile.loadActive(null)).thenReturn(profile);
                    OpMode op = mode == 0 ? new LimelightClusterTrackingTestTeleOp()
                            : mode == 1 ? new LimelightAllianceClusterAdapterTestTeleOp()
                            : new LimelightAprilTagTestTeleOp();
                    op.hardwareMap = mock(HardwareMap.class);
                    op.telemetry = mock(Telemetry.class);
                    op.gamepad1 = mock(Gamepad.class);
                    Limelight3A camera = mock(Limelight3A.class);
                    when(op.hardwareMap.get(Limelight3A.class, profile.limelightName)).thenReturn(camera);
                    op.init();
                    profiles.verify(() -> RobotProfile.loadActive(null));
                    verify(op.telemetry).addData("Robot profile", profile.name);
                    if (mode == 2) {
                        verify(camera).pipelineSwitch(profile.limelightPipeline);
                    } else {
                        verify(visions.constructed().get(0)).init(op.hardwareMap, profile);
                    }
                    if (mode == 0) {
                        verify(drives.constructed().get(0)).init(op.hardwareMap, profile);
                    }
                    clearInvocations(op.telemetry);
                    op.init_loop();
                    verify(op.telemetry).addData("Robot profile", profile.name);
                    op.stop();
                }
            }
        }
    }
}
