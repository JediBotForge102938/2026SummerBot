package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control.DriveModePreferences;
import org.firstinspires.ftc.teamcode.control.TeleopController;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ConfigurationSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotProfile;
import org.firstinspires.ftc.teamcode.subsystems.LauncherFeeder;
import org.firstinspires.ftc.teamcode.control.MechanismCoordinator;
import org.firstinspires.ftc.teamcode.BioBuzzLimelightClusterAdapter.Alliance;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.telemetry.StarterBotTelemetry;
import java.util.List;
import org.junit.Test;
import org.mockito.MockedConstruction;
import org.mockito.MockedStatic;

public class BioBuzzCompetitionTeleopTest {
    @Test
    public void competitionUsesSharedControllerForInitLoopDriveLoopAndStop() {
        RobotProfile profile = TestProfiles.starter();
        try (MockedStatic<RobotProfile> profiles = mockStatic(RobotProfile.class);
                MockedConstruction<DriveSubsystem> drives = mockConstruction(DriveSubsystem.class);
                MockedConstruction<IntakeSubsystem> intakes = mockConstruction(IntakeSubsystem.class,
                        (mock, context) -> assertEquals(false, context.arguments().get(0)));
                MockedConstruction<LauncherSubsystem> launchers = mockConstruction(LauncherSubsystem.class,
                        (mock, context) -> {
                            List<?> motors = (List<?>) context.arguments().get(0);
                            assertEquals(2, motors.size());
                            LauncherSubsystem.MotorConfig left =
                                    (LauncherSubsystem.MotorConfig) motors.get(0);
                            LauncherSubsystem.MotorConfig right =
                                    (LauncherSubsystem.MotorConfig) motors.get(1);
                            assertEquals("left_launcher", left.name);
                            assertEquals(DcMotor.Direction.REVERSE, left.direction);
                            assertEquals("right_launcher", right.name);
                            assertEquals(DcMotor.Direction.FORWARD, right.direction);
                        });
                MockedConstruction<VisionSubsystem> visions = mockConstruction(VisionSubsystem.class);
                MockedConstruction<DriveModePreferences> preferences =
                        mockConstruction(DriveModePreferences.class, (mock, context) -> {
                            when(mock.getConfigurationRedAlliance()).thenReturn(true);
                            when(mock.getConfigurationFieldCentric()).thenReturn(true);
                        });
                MockedConstruction<MechanismCoordinator> mechanisms =
                        mockConstruction(MechanismCoordinator.class,
                                (mock, context) -> assertSame(LauncherFeeder.DISABLED,
                                        context.arguments().get(2)));
                MockedConstruction<TeleopController> controllers =
                        mockConstruction(TeleopController.class);
                MockedConstruction<StarterBotTelemetry> displays =
                        mockConstruction(StarterBotTelemetry.class,
                                (mock, context) -> assertEquals(false, context.arguments().get(5)))) {
            profiles.when(() -> RobotProfile.loadActive("competitionbot")).thenReturn(profile);
            CompetitionBotTeleOp opMode = new CompetitionBotTeleOp();
            opMode.hardwareMap = mock(HardwareMap.class);
            opMode.gamepad1 = mock(Gamepad.class);
            opMode.telemetry = mock(Telemetry.class);
            opMode.init();

            TeleopController controller = controllers.constructed().get(0);
            verify(drives.constructed().get(0)).init(opMode.hardwareMap, profile);
            verify(intakes.constructed().get(0)).init(opMode.hardwareMap);
            verify(launchers.constructed().get(0)).init(opMode.hardwareMap);
            verify(visions.constructed().get(0)).init(opMode.hardwareMap, profile);
            verifyNoInteractions(controller, displays.constructed().get(0));
            opMode.gamepad1.dpad_right = true;
            opMode.init_loop();
            opMode.gamepad1.dpad_right = false;
            opMode.init_loop();
            opMode.gamepad1.dpad_up = true;
            opMode.init_loop();
            opMode.gamepad1.dpad_up = false;
            opMode.init_loop();
            opMode.gamepad1.dpad_right = true;
            opMode.init_loop();
            verifyNoInteractions(controller, displays.constructed().get(0));
            opMode.start();
            org.mockito.ArgumentCaptor<ConfigurationSubsystem> selected =
                    org.mockito.ArgumentCaptor.forClass(ConfigurationSubsystem.class);
            verify(controller).configure(selected.capture(), eq(opMode.gamepad1));
            assertTrue(selected.getValue().isLocked());
            assertTrue(selected.getValue().isFieldCentric());
            assertEquals(Alliance.BLUE, selected.getValue().getAlliance());
            assertEquals(ConfigurationSubsystem.StartingSpot.SCORING,
                    selected.getValue().getStartingSpot());
            verify(drives.constructed().get(0)).resetHeading(270, 180);
            opMode.start();
            verify(drives.constructed().get(0)).resetHeading(270, 180);
            opMode.loop();
            verify(controller).update(opMode.gamepad1);
            verify(controller, never()).updateInit(any());
            verify(drives.constructed().get(0), never()).resetHeading();
            opMode.stop();
            verify(controller).stop();
            verify(opMode.gamepad1).stopRumble();
            verify(visions.constructed().get(0)).stop();
            verify(displays.constructed().get(0), times(4)).publish();
        }
    }

    @Test
    public void stopAfterPartialInitializationStillStopsInitializedHardware() {
        RobotProfile profile = TestProfiles.starter();
        try (MockedStatic<RobotProfile> profiles = mockStatic(RobotProfile.class);
                MockedConstruction<DriveSubsystem> drives = mockConstruction(DriveSubsystem.class);
                MockedConstruction<IntakeSubsystem> intakes = mockConstruction(IntakeSubsystem.class);
                MockedConstruction<LauncherSubsystem> launchers =
                        mockConstruction(LauncherSubsystem.class,
                                (mock, context) -> doThrow(new IllegalStateException("Launcher init failed"))
                                        .when(mock).init(any()));
                MockedConstruction<VisionSubsystem> visions = mockConstruction(VisionSubsystem.class)) {
            profiles.when(() -> RobotProfile.loadActive("competitionbot")).thenReturn(profile);
            CompetitionBotTeleOp opMode = new CompetitionBotTeleOp();
            opMode.hardwareMap = mock(HardwareMap.class);
            opMode.gamepad1 = mock(Gamepad.class);
            assertThrows(IllegalStateException.class, opMode::init);
            opMode.stop();
            verify(drives.constructed().get(0)).driveRobotRelative(0, 0, 0);
            verify(intakes.constructed().get(0)).setPower(0);
            verify(visions.constructed().get(0)).stop();
        }
    }
}
