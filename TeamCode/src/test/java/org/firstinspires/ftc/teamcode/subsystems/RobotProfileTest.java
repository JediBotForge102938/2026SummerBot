package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import com.qualcomm.ftccommon.configuration.RobotConfigFile;
import com.qualcomm.ftccommon.configuration.RobotConfigFileManager;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.control.OTOSFieldNavigator;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;
import org.mockito.MockedConstruction;
import java.io.File;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;

public class RobotProfileTest {
    @Rule public TemporaryFolder folder = new TemporaryFolder();

    private static final String VALID = "{"
            + "\"schemaVersion\":1,\"robot\":\"starterbot\","
            + "\"chassis\":{\"lengthCm\":45.72,\"widthCm\":45.72},"
            + "\"imu\":{\"name\":\"custom-imu\",\"logoFacing\":\"UP\",\"usbFacing\":\"FORWARD\"},"
            + "\"limelight\":{\"name\":\"camera\",\"pipeline\":3,\"aimDirection\":\"launcher\","
            + "\"lensFromBackCm\":9.5,\"lensFromLeftCm\":22.86,\"lensHeightCm\":31.8,"
            + "\"yawDegrees\":180,\"pitchUpDegrees\":37,\"rollDegrees\":0},"
            + "\"otos\":{\"name\":\"optical\",\"rawAxes\":"
            + "\"forward-negative-x-right-positive-y-clockwise-negative\","
            + "\"opticalCenterFromBackCm\":20,\"opticalCenterFromLeftCm\":10,"
            + "\"linearScalar\":1.02,\"angularScalar\":0.98}}";

    private RobotProfile load(String json) throws Exception {
        File file = new File(folder.getRoot(), "my-bot.profile.json");
        Files.write(file.toPath(), json.getBytes(StandardCharsets.UTF_8));
        return RobotProfile.load(folder.getRoot(), "my-bot", "starterbot");
    }

    @Test
    public void currentProfilesArePaired() {
        File directory = new File("../robot-configs");
        RobotProfile starter = RobotProfile.load(directory, "biobuzz-starterbot", "starterbot");
        RobotProfile competition = RobotProfile.load(directory, "biobuzz-competitionbot", "competitionbot");
        assertEquals(19, starter.otosBackCm, 0);
        assertEquals(7.5, starter.otosLeftCm, 0);
        assertEquals("limelight", starter.limelightName);
        assertEquals("competitionbot", competition.robot);
        assertEquals(45.72, starter.chassisLengthCm, 0);
        assertEquals(45.72, starter.chassisWidthCm, 0);
        assertEquals(9.5, starter.limelightFromBackCm, 0);
        assertEquals(22.86, starter.limelightFromLeftCm, 0);
        assertEquals(31.8, starter.limelightHeightCm, 0);
        assertEquals(180, starter.limelightYawDegrees, 0);
        assertEquals(37, starter.limelightPitchUpDegrees, 0);
        assertEquals(0, starter.limelightRollDegrees, 0);
        assertThrows(IllegalStateException.class,
                () -> RobotProfile.load(directory, "biobuzz-starterbot", "competitionbot"));
    }

    @Test
    public void rejectsMissingInvalidAndUnsupportedSettingsWithoutFallback() throws Exception {
        assertThrows(IllegalStateException.class,
                () -> RobotProfile.load(folder.getRoot(), "../unsafe", "starterbot"));
        assertThrows(IllegalStateException.class,
                () -> RobotProfile.load(folder.getRoot(), "missing", "starterbot"));
        for (String invalid : new String[] {
                "{}",
                VALID.replace("\"schemaVersion\":1", "\"schemaVersion\":2"),
                VALID.replace("\"pipeline\":3", "\"pipeline\":11"),
                VALID.replace("\"pipeline\":3", "\"pipeline\":\"3\""),
                VALID.replace("\"usbFacing\":\"FORWARD\"", "\"usbFacing\":\"DOWN\""),
                VALID.replace("\"linearScalar\":1.02", "\"linearScalar\":2"),
                VALID.replace("\"lengthCm\":45.72", "\"lengthCm\":0"),
                VALID.replace("\"lensHeightCm\":31.8", "\"lensHeightCm\":-1"),
                VALID.replace("\"aimDirection\":\"launcher\"", "\"aimDirection\":\"forward\""),
                VALID.replace("\"opticalCenterFromBackCm\":20", "\"opticalCenterFromBackCm\":-1"),
                VALID.replace("\"robot\":\"starterbot\"", "\"robot\":\"competitionbot\""),
                VALID.replace("\"pipeline\":3", "\"pipeline\":3,\"mountYaw\":90")
        }) {
            assertThrows(invalid, IllegalStateException.class, () -> load(invalid));
        }
    }

    @Test
    public void profileSettingsConfigureActualSubsystemsAndNavigator() throws Exception {
        RobotProfile profile = load(VALID);
        HardwareMap hardware = mock(HardwareMap.class);
        when(hardware.get(eq(DcMotor.class), anyString())).thenAnswer(i -> mock(DcMotor.class));
        IMU imu = mock(IMU.class);
        when(hardware.get(IMU.class, "custom-imu")).thenReturn(imu);
        when(imu.initialize(any())).thenReturn(true);
        Limelight3A camera = mock(Limelight3A.class);
        when(hardware.get(Limelight3A.class, "camera")).thenReturn(camera);
        try (MockedConstruction<RevHubOrientationOnRobot> orientation =
                     mockConstruction(RevHubOrientationOnRobot.class, (mock, context) -> {
                         assertEquals(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                 context.arguments().get(0));
                         assertEquals(RevHubOrientationOnRobot.UsbFacingDirection.FORWARD,
                                 context.arguments().get(1));
                     })) {
            new DriveSubsystem().init(hardware, profile);
            verify(imu).initialize(any());
            assertEquals(1, orientation.constructed().size());
        }
        VisionSubsystem vision = new VisionSubsystem();
        vision.init(hardware, profile);
        verify(camera).pipelineSwitch(3);
        verify(camera).start();
        assertEquals("camera", vision.getCameraName());
        assertEquals(3, vision.getPipeline());
        assertEquals("optical", profile.otosName);
        assertEquals(1.02, profile.otosLinearScalar, 0);
        assertEquals(0.98, profile.otosAngularScalar, 0);
        OTOSFieldNavigator navigator = new OTOSFieldNavigator(-1, -2);
        navigator.updatePose(0, 0, 90, 0, 0, 0);
        assertEquals(OTOSFieldNavigator.START_X + 3, navigator.getCenter().x, 1e-9);
        assertEquals(1, navigator.getCenter().y, 1e-9);
    }

    @Test
    public void activeSelectionUsesTheSdkConfigurationName() {
        try (MockedConstruction<RobotConfigFileManager> managers =
                     mockConstruction(RobotConfigFileManager.class, (mock, context) -> {
                         RobotConfigFile selected = mock(RobotConfigFile.class);
                         when(selected.getName()).thenReturn("missing-profile");
                         when(mock.getActiveConfig()).thenReturn(selected);
                     })) {
            IllegalStateException error = assertThrows(IllegalStateException.class,
                    () -> RobotProfile.loadActive("starterbot"));
            assertTrue(error.getMessage().contains("missing-profile"));
            verify(managers.constructed().get(0)).getActiveConfig();
        }
    }
}
