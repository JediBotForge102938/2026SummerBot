package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import com.qualcomm.ftccommon.configuration.RobotConfigFile;
import com.qualcomm.ftccommon.configuration.RobotConfigFileManager;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
            + "\"schemaVersion\":8,\"robot\":\"starterbot\","
            + "\"chassis\":{\"lengthCm\":45.72,\"widthCm\":45.72},"
            + "\"drive\":{\"mirrorStrafe\":false},"
            + "\"intake\":{\"motorDirection\":\"FORWARD\",\"defaultSpeedPercent\":100,\"feedAssistPercent\":50},"
            + "\"launcher\":{\"feedVelocityMarginTicksPerSecond\":50,"
            + "\"pidf\":{\"p\":40,\"i\":0,\"d\":0,\"f\":12.5},"
            + "\"motors\":[{\"name\":\"launcher\",\"direction\":\"FORWARD\"}]},"
            + "\"imu\":{\"name\":\"custom-imu\",\"logoFacing\":\"UP\",\"usbFacing\":\"FORWARD\"},"
            + "\"limelight\":{\"name\":\"camera\",\"pipeline\":3,\"aimDirection\":\"launcher\","
            + "\"lensFromBackCm\":9.5,\"lensFromLeftCm\":22.86,\"lensHeightCm\":31.8,"
            + "\"yawDegrees\":180,\"pitchUpDegrees\":37,\"rollDegrees\":0},"
            + "\"otos\":{\"name\":\"optical\",\"rawAxes\":"
            + "\"forward-negative-x-right-positive-y-clockwise-negative\","
            + "\"opticalCenterFromBackCm\":20,\"opticalCenterFromLeftCm\":10,"
            + "\"linearScalar\":1.02,\"angularScalar\":0.98},"
            + "\"navigation\":{\"geometryVerified\":false,\"pedroTuningVerified\":false,"
            + "\"simulatorEnabled\":true,\"transitClearanceIn\":2,\"wallClearanceIn\":2,"
            + "\"startWallPlacementToleranceIn\":1,"
            + "\"flowerStandoffIn\":3,\"curveInsetIn\":6,\"transitPower\":0.5,\"finalApproachPower\":0.2,"
            + "\"arrivalToleranceIn\":1,\"precisionDistanceIn\":10,"
            + "\"headingToleranceDegrees\":3,"
            + "\"stationarySpeedInPerSec\":0.75,"
            + "\"stationaryTurnRateDegreesPerSec\":5,\"settleSeconds\":0.2,"
            + "\"timeoutSeconds\":60,\"progressTimeoutSeconds\":4,"
            + "\"progressThresholdIn\":0.1,\"translationalP\":0.08,"
            + "\"translationalD\":0,\"headingP\":1,\"headingD\":0,"
            + "\"driveP\":0.05,\"driveD\":0,\"forwardVelocity\":25,"
            + "\"lateralVelocity\":20}}";

    private RobotProfile load(String json) throws Exception {
        File file = new File(folder.getRoot(), "my-bot.profile.json");
        Files.write(file.toPath(), json.getBytes(StandardCharsets.UTF_8));
        return RobotProfile.load(folder.getRoot(), "my-bot", "starterbot");
    }

    @Test public void launcherMotorDirectionsAreLoadedFromProfile() throws Exception {
        RobotProfile normal = load(VALID);
        assertEquals(1, normal.launcherMotors.size());
        assertEquals("launcher", normal.launcherMotors.get(0).name);
        assertEquals(DcMotorSimple.Direction.FORWARD, normal.launcherMotors.get(0).direction);
        RobotProfile reversed = load(VALID.replace("\"direction\":\"FORWARD\"",
                "\"direction\":\"REVERSE\""));
        assertEquals(DcMotorSimple.Direction.REVERSE, reversed.launcherMotors.get(0).direction);
        assertThrows(UnsupportedOperationException.class, () -> normal.launcherMotors.clear());
        RobotProfile competition = RobotProfile.load(new File("../robot-configs"),
                "biobuzz-competitionbot", "competitionbot");
        assertEquals(1, competition.launcherMotors.size());
        assertEquals("launcher", competition.launcherMotors.get(0).name);
        assertEquals(DcMotorSimple.Direction.FORWARD, competition.launcherMotors.get(0).direction);
        assertEquals(40, normal.launcherPidf.p, 0);
        assertEquals(0, normal.launcherPidf.i, 0);
        assertEquals(0, competition.launcherPidf.d, 0);
        assertEquals(12.5, competition.launcherPidf.f, 0);
        assertEquals(50, normal.launcherFeedVelocityMargin);
        assertEquals(250, competition.launcherFeedVelocityMargin);
        assertNull(normal.launcherGate);
        assertEquals("gate", competition.launcherGate.name);
        assertEquals(0.0, competition.launcherGate.openPosition, 0);
        assertEquals(0.3, competition.launcherGate.closedPosition, 0);
        assertEquals(2.5, load(VALID.replace("\"i\":0", "\"i\":2.5")).launcherPidf.i, 0);
    }

    @Test public void legacyProfilesRemainUsableButCannotEnableLandmarkTravel() throws Exception {
        com.google.gson.JsonObject legacy = new com.google.gson.Gson()
                .fromJson(VALID, com.google.gson.JsonObject.class);
        legacy.remove("navigation");
        RobotProfile profile = load(legacy.toString());
        assertEquals("starterbot", profile.robot);
        assertNull(profile.navigation);
        assertFalse(profile.isAutonomousReady());
        assertThrows(IllegalStateException.class,
                () -> new org.firstinspires.ftc.teamcode.control.LandmarkFieldMap(profile));
    }

    @Test
    public void currentProfilesArePaired() {
        File directory = new File("../robot-configs");
        RobotProfile starter = RobotProfile.load(directory, "biobuzz-starterbot", "starterbot");
        RobotProfile competition = RobotProfile.load(directory, "biobuzz-competitionbot", "competitionbot");
        assertEquals(18.8, starter.otosBackCm, 0);
        assertEquals(7.5, starter.otosLeftCm, 0);
        assertEquals("limelight", starter.limelightName);
        assertEquals("competitionbot", competition.robot);
        assertEquals(DcMotorSimple.Direction.FORWARD, starter.intakeMotorDirection);
        assertEquals(DcMotorSimple.Direction.REVERSE, competition.intakeMotorDirection);
        assertFalse(starter.mirrorStrafe);
        assertTrue(competition.mirrorStrafe);
        assertEquals(45.72, starter.chassisLengthCm, 0);
        assertEquals(45.72, starter.chassisWidthCm, 0);
        assertEquals(9.5, starter.limelightFromBackCm, 0);
        assertEquals(22.86, starter.limelightFromLeftCm, 0);
        assertEquals(31.8, starter.limelightHeightCm, 0);
        assertEquals(180, starter.limelightYawDegrees, 0);
        assertEquals(37, starter.limelightPitchUpDegrees, 0);
        assertEquals(0, starter.limelightRollDegrees, 0);
        assertTrue(starter.navigation.geometryVerified);
        assertTrue(starter.navigation.pedroTuningVerified);
        assertTrue(starter.navigation.simulatorEnabled);
        assertEquals(0.5, starter.navigation.transitPower, 0);
        assertEquals(0.2, starter.navigation.finalApproachPower, 0);
        assertEquals(60, starter.navigation.timeoutSeconds, 0);
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
                VALID.replace("\"schemaVersion\":8", "\"schemaVersion\":7"),
                VALID.replace("\"drive\":{\"mirrorStrafe\":false},", ""),
                VALID.replace("\"mirrorStrafe\":false", "\"mirrorStrafe\":\"false\""),
                VALID.replace("\"defaultSpeedPercent\":100", "\"defaultSpeedPercent\":101"),
                VALID.replace("\"defaultSpeedPercent\":100", "\"defaultSpeedPercent\":74"),
                VALID.replace("\"feedAssistPercent\":50", "\"feedAssistPercent\":-1"),
                VALID.replace("\"feedAssistPercent\":50", "\"feedAssistPercent\":101"),
                VALID.replace(",\"feedAssistPercent\":50", ""),
                VALID.replace("\"feedVelocityMarginTicksPerSecond\":50,", ""),
                VALID.replace("\"feedVelocityMarginTicksPerSecond\":50", "\"feedVelocityMarginTicksPerSecond\":-1"),
                VALID.replace("\"feedVelocityMarginTicksPerSecond\":50", "\"feedVelocityMarginTicksPerSecond\":6001"),
                VALID.replace("\"feedVelocityMarginTicksPerSecond\":50", "\"feedVelocityMarginTicksPerSecond\":2.5"),
                VALID.replace("\"feedVelocityMarginTicksPerSecond\":50", "\"feedVelocityMarginTicksPerSecond\":\"50\""),
                VALID.replace("\"pidf\":{\"p\":40,\"i\":0,\"d\":0,\"f\":12.5},", ""),
                VALID.replace("\"p\":40", "\"p\":-1"),
                VALID.replace("\"i\":0", "\"i\":\"0\""),
                VALID.replace("\"f\":12.5", "\"f\":1e999"),
                VALID.replace("\"d\":0,", ""),
                VALID.replace("\"motors\":[{\"name\":\"launcher\",\"direction\":\"FORWARD\"}]", "\"motors\":[]"),
                VALID.replace("\"motors\":[{\"name\":\"launcher\",\"direction\":\"FORWARD\"}]",
                        "\"motors\":[{\"name\":\"launcher\",\"direction\":\"FORWARD\"},{\"name\":\"launcher\",\"direction\":\"REVERSE\"}]"),
                VALID.replace("\"direction\":\"FORWARD\"", "\"direction\":\"INVALID\""),
                VALID.replace("\"motors\":[{\"name\":\"launcher\",\"direction\":\"FORWARD\"}]", "\"motors\":[null]"),
                VALID.replace("\"pipeline\":3", "\"pipeline\":11"),
                VALID.replace("\"pipeline\":3", "\"pipeline\":\"3\""),
                VALID.replace("\"usbFacing\":\"FORWARD\"", "\"usbFacing\":\"DOWN\""),
                VALID.replace("\"linearScalar\":1.02", "\"linearScalar\":2"),
                VALID.replace("\"lengthCm\":45.72", "\"lengthCm\":0"),
                VALID.replace("\"lensHeightCm\":31.8", "\"lensHeightCm\":-1"),
                VALID.replace("\"aimDirection\":\"launcher\"", "\"aimDirection\":\"forward\""),
                VALID.replace("\"motorDirection\":\"FORWARD\"", "\"motorDirection\":\"SIDEWAYS\""),
                VALID.replace("\"opticalCenterFromBackCm\":20", "\"opticalCenterFromBackCm\":-1"),
                VALID.replace("\"robot\":\"starterbot\"", "\"robot\":\"competitionbot\""),
                VALID.replace("\"pipeline\":3", "\"pipeline\":3,\"mountYaw\":90")
        }) {
            assertThrows(invalid, IllegalStateException.class, () -> load(invalid));
        }
    }

    @Test public void competitionGateRequiresValidDistinctPositions() throws Exception {
        String competition = VALID.replace("\"robot\":\"starterbot\"", "\"robot\":\"competitionbot\"")
            .replace("\"launcher\":{", "\"launcher\":{\"gate\":{\"name\":\"gate\","
                    + "\"openPosition\":0.6,\"closedPosition\":0.3},");
        File file = new File(folder.getRoot(), "competition.profile.json");
        for (String invalid : new String[] {
            competition.replace("\"gate\":{\"name\":\"gate\",\"openPosition\":0.6,\"closedPosition\":0.3},", ""),
            competition.replace("\"openPosition\":0.6", "\"openPosition\":1.1"),
            competition.replace("\"closedPosition\":0.3", "\"closedPosition\":-0.1"),
            competition.replace("\"closedPosition\":0.3", "\"closedPosition\":0.6"),
            competition.replace("\"openPosition\":0.6", "\"openPosition\":\"0.6\""),
            competition.replace("\"closedPosition\":0.3", "\"closedPosition\":1e999")
        }) {
        Files.write(file.toPath(), invalid.getBytes(StandardCharsets.UTF_8));
        assertThrows(invalid, IllegalStateException.class,
                () -> RobotProfile.load(folder.getRoot(), "competition", "competitionbot"));
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
