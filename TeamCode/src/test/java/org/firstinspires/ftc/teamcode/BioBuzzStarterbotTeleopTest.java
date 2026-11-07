package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BioBuzzLimelightClusterAdapter.Alliance;
import org.firstinspires.ftc.teamcode.control.DriveModePreferences;
import org.firstinspires.ftc.teamcode.control.HiveTrackingController;
import org.firstinspires.ftc.teamcode.control.TeleopController;
import org.firstinspires.ftc.teamcode.subsystems.ConfigurationSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WindmillSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotProfile;
import org.firstinspires.ftc.teamcode.telemetry.StarterBotTelemetry;
import org.junit.Test;
import org.mockito.ArgumentCaptor;
import org.mockito.MockedConstruction;
import org.mockito.MockedStatic;

public class BioBuzzStarterbotTeleopTest {
    @Test
    public void missingProfileStopsInitBeforeAnyRobotHardwareIsInitialized() {
        try (MockedStatic<RobotProfile> profiles = mockStatic(RobotProfile.class);
                MockedConstruction<DriveSubsystem> drives = mockConstruction(DriveSubsystem.class);
                MockedConstruction<IntakeSubsystem> intakes = mockConstruction(IntakeSubsystem.class);
                MockedConstruction<LauncherSubsystem> launchers = mockConstruction(LauncherSubsystem.class);
                MockedConstruction<WindmillSubsystem> windmills = mockConstruction(WindmillSubsystem.class);
                MockedConstruction<VisionSubsystem> visions = mockConstruction(VisionSubsystem.class)) {
            profiles.when(() -> RobotProfile.loadActive("starterbot"))
                    .thenThrow(new IllegalStateException("Missing profile"));
            StarterBotTeleOp opMode = new StarterBotTeleOp();
            assertThrows(IllegalStateException.class, opMode::init);
            verifyNoInteractions(drives.constructed().get(0), intakes.constructed().get(0),
                    launchers.constructed().get(0), windmills.constructed().get(0),
                    visions.constructed().get(0));
        }
    }

    @Test
    public void configurationReachesStarterComponentsOnlyAtStart() {
        RobotProfile profile = TestProfiles.starter();
        try (MockedStatic<RobotProfile> profiles = mockStatic(RobotProfile.class);
                MockedConstruction<DriveSubsystem> drives = mockConstruction(DriveSubsystem.class);
                MockedConstruction<IntakeSubsystem> intakes = mockConstruction(IntakeSubsystem.class);
                MockedConstruction<LauncherSubsystem> launchers = mockConstruction(LauncherSubsystem.class);
                MockedConstruction<WindmillSubsystem> windmills = mockConstruction(WindmillSubsystem.class);
                MockedConstruction<VisionSubsystem> visions = mockConstruction(VisionSubsystem.class);
                MockedConstruction<DriveModePreferences> preferences = mockConstruction(DriveModePreferences.class,
                        (mock, context) -> {
                            when(mock.getConfigurationRedAlliance()).thenReturn(true);
                            when(mock.getConfigurationFieldCentric()).thenReturn(true);
                        });
                MockedConstruction<HiveTrackingController> tracking = mockConstruction(HiveTrackingController.class);
                MockedConstruction<TeleopController> controllers = mockConstruction(TeleopController.class);
                MockedConstruction<StarterBotTelemetry> displays = mockConstruction(StarterBotTelemetry.class)) {
            profiles.when(() -> RobotProfile.loadActive("starterbot")).thenReturn(profile);
            StarterBotTeleOp opMode = new StarterBotTeleOp();
            opMode.hardwareMap = mock(HardwareMap.class);
            opMode.gamepad1 = mock(Gamepad.class);
            opMode.telemetry = mock(Telemetry.class);
            opMode.init();
            TeleopController controller = controllers.constructed().get(0);
            DriveSubsystem drive = drives.constructed().get(0);
            opMode.gamepad1.left_stick_y = -1;
            right(opMode);
            down(opMode);
            right(opMode);
            down(opMode);
            right(opMode);
            verifyNoInteractions(controller, displays.constructed().get(0));
            verify(drive).init(opMode.hardwareMap, profile);
            verifyNoMoreInteractions(drive);
            opMode.start();
            opMode.start();
            verify(drive).resetHeading(270, 180);
            ArgumentCaptor<ConfigurationSubsystem> selected =
                    ArgumentCaptor.forClass(ConfigurationSubsystem.class);
            verify(controller).configure(selected.capture(), eq(opMode.gamepad1));
            assertTrue(selected.getValue().isLocked());
            assertEquals(Alliance.BLUE, selected.getValue().getAlliance());
            assertFalse(selected.getValue().isFieldCentric());
            assertEquals(ConfigurationSubsystem.StartingSpot.SCORING,
                    selected.getValue().getStartingSpot());
            opMode.loop();
            verify(controller).update(opMode.gamepad1);
            verify(controller, never()).updateInit(any());
            opMode.stop();
            verify(controller).stop();
            verify(visions.constructed().get(0)).stop();
        }
    }

    private static void down(StarterBotTeleOp opMode) {
        opMode.gamepad1.dpad_down = true;
        opMode.init_loop();
        opMode.gamepad1.dpad_down = false;
        opMode.init_loop();
    }

    private static void right(StarterBotTeleOp opMode) {
        opMode.gamepad1.dpad_right = true;
        opMode.init_loop();
        opMode.gamepad1.dpad_right = false;
        opMode.init_loop();
    }
}
