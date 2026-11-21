package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control.DriveModePreferences;
import org.firstinspires.ftc.teamcode.control.TeleopController;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ConfigurationSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotProfile;
import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;
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
        RobotProfile profile = RobotProfile.load(new java.io.File("../robot-configs"),
                "biobuzz-competitionbot", "competitionbot");
        try (MockedStatic<RobotProfile> profiles = mockStatic(RobotProfile.class);
                MockedConstruction<DriveSubsystem> drives = mockConstruction(DriveSubsystem.class);
                MockedConstruction<GateSubsystem> gates = mockConstruction(GateSubsystem.class,
                        (mock, context) -> assertSame(profile.launcherGate, context.arguments().get(0)));
                MockedConstruction<IntakeSubsystem> intakes = mockConstruction(IntakeSubsystem.class,
                        (mock, context) -> {
                            assertEquals(false, context.arguments().get(0));
                            assertSame(profile, context.arguments().get(1));
                        });
                MockedConstruction<LauncherSubsystem> launchers = mockConstruction(LauncherSubsystem.class,
                        (mock, context) -> {
                            List<?> motors = (List<?>) context.arguments().get(0);
                            assertSame(profile.launcherMotors, motors);
                            assertSame(profile.launcherPidf, context.arguments().get(1));
                            assertEquals(profile.launcherFeedVelocityMargin,
                                    context.arguments().get(2));
                            assertEquals(1, motors.size());
                            LauncherSubsystem.MotorConfig launcher =
                                    (LauncherSubsystem.MotorConfig) motors.get(0);
                            assertEquals("launcher", launcher.name);
                            assertEquals(profile.launcherMotors.get(0).direction, launcher.direction);
                        });
                MockedConstruction<VisionSubsystem> visions = mockConstruction(VisionSubsystem.class);
                MockedConstruction<DriveModePreferences> preferences =
                        mockConstruction(DriveModePreferences.class, (mock, context) -> {
                            when(mock.getConfigurationRedAlliance()).thenReturn(true);
                            when(mock.getConfigurationFieldCentric()).thenReturn(true);
                        });
                MockedConstruction<MechanismCoordinator> mechanisms =
                        mockConstruction(MechanismCoordinator.class,
                                (mock, context) -> {
                                    assertSame(LauncherFeeder.DISABLED, context.arguments().get(2));
                                    assertEquals(profile.intakeFeedAssistPercent,
                                            context.arguments().get(3));
                                });
                MockedConstruction<TeleopController> controllers =
                        mockConstruction(TeleopController.class);
                MockedConstruction<StarterBotTelemetry> displays =
                        mockConstruction(StarterBotTelemetry.class)) {
            profiles.when(() -> RobotProfile.loadActive("competitionbot")).thenReturn(profile);
            CompetitionBotTeleOp opMode = new CompetitionBotTeleOp();
            opMode.hardwareMap = mock(HardwareMap.class);
            opMode.gamepad1 = mock(Gamepad.class);
            opMode.telemetry = mock(Telemetry.class);
            opMode.init();

            TeleopController controller = controllers.constructed().get(0);
            verify(controller).setIntakeProfile(profile.robot, profile.intakeDefaultSpeedPercent);
            clearInvocations(controller);
            verify(drives.constructed().get(0)).init(opMode.hardwareMap, profile);
            verify(gates.constructed().get(0)).init(opMode.hardwareMap);
            verify(intakes.constructed().get(0)).init(opMode.hardwareMap);
            verify(launchers.constructed().get(0)).init(opMode.hardwareMap);
            verify(visions.constructed().get(0)).init(opMode.hardwareMap, profile);
            verifyNoInteractions(controller, displays.constructed().get(0));
            opMode.gamepad1.dpad_right = true;
            opMode.init_loop();
            opMode.gamepad1.dpad_right = false;
            opMode.init_loop();
            opMode.gamepad1.dpad_down = true;
            opMode.init_loop();
            opMode.gamepad1.dpad_down = false;
            opMode.init_loop();
            opMode.gamepad1.dpad_down = true;
            opMode.init_loop();
            opMode.gamepad1.dpad_down = false;
            opMode.init_loop();
            opMode.gamepad1.dpad_right = true;
            opMode.init_loop();
            verifyNoInteractions(controller, displays.constructed().get(0));
            opMode.gamepad1.b = true;
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
            verify(gates.constructed().get(0), never()).update(anyBoolean());
            opMode.gamepad1.b = false;
            opMode.loop();
            opMode.gamepad1.b = true;
            opMode.loop();
            opMode.loop();
            verify(gates.constructed().get(0), times(1)).update(true);
            opMode.gamepad1.b = false;
            opMode.loop();
            opMode.gamepad1.b = true;
            opMode.loop();
            verify(gates.constructed().get(0), times(1)).update(false);
            opMode.gamepad1.b = false;
            opMode.loop();
            opMode.gamepad1.b = true;
            opMode.loop();
            verify(gates.constructed().get(0), times(2)).update(true);
            verify(controller, times(8)).update(opMode.gamepad1);
            verify(controller, never()).updateInit(any());
            verify(drives.constructed().get(0), never()).resetHeading();
            opMode.stop();
            verify(gates.constructed().get(0), times(2)).update(false);
            verify(controller).stop();
            verify(opMode.gamepad1).stopRumble();
            verify(visions.constructed().get(0)).stop();
            verify(displays.constructed().get(0), times(11)).publish();
        }
    }

    @Test
    public void gateTogglesDuringInitAndRetainsPositionAcrossStart() {
        RobotProfile profile = RobotProfile.load(new java.io.File("../robot-configs"),
                "biobuzz-competitionbot", "competitionbot");
        try (MockedStatic<RobotProfile> profiles = mockStatic(RobotProfile.class);
                MockedConstruction<GateSubsystem> gates = mockConstruction(GateSubsystem.class);
                MockedConstruction<DriveSubsystem> drives = mockConstruction(DriveSubsystem.class);
                MockedConstruction<IntakeSubsystem> intakes = mockConstruction(IntakeSubsystem.class);
                MockedConstruction<LauncherSubsystem> launchers = mockConstruction(LauncherSubsystem.class);
                MockedConstruction<VisionSubsystem> visions = mockConstruction(VisionSubsystem.class);
                MockedConstruction<DriveModePreferences> preferences = mockConstruction(DriveModePreferences.class);
                MockedConstruction<MechanismCoordinator> mechanisms = mockConstruction(MechanismCoordinator.class);
                MockedConstruction<TeleopController> controllers = mockConstruction(TeleopController.class);
                MockedConstruction<StarterBotTelemetry> displays = mockConstruction(StarterBotTelemetry.class)) {
            profiles.when(() -> RobotProfile.loadActive("competitionbot")).thenReturn(profile);
            CompetitionBotTeleOp opMode = new CompetitionBotTeleOp();
            opMode.hardwareMap = mock(HardwareMap.class);
            opMode.gamepad1 = mock(Gamepad.class);
            opMode.telemetry = mock(Telemetry.class);
            opMode.gamepad1.b = true;
            opMode.init();
            GateSubsystem gate = gates.constructed().get(0);
            clearInvocations(controllers.constructed().get(0));
            verify(gate).init(opMode.hardwareMap);
            opMode.init_loop();
            verify(gate, never()).update(anyBoolean());
            opMode.gamepad1.b = false;
            opMode.init_loop();
            opMode.gamepad1.b = true;
            opMode.init_loop();
            opMode.init_loop();
            verify(gate).update(true);
            verifyNoInteractions(controllers.constructed().get(0));
            opMode.start();
            opMode.loop();
            verify(gate, never()).update(false);
            opMode.gamepad1.b = false;
            opMode.loop();
            opMode.gamepad1.b = true;
            opMode.loop();
            verify(gate).update(false);
            opMode.stop();
            verify(gate, times(2)).update(false);
        }
    }

    @Test
    public void gateFollowsControllerSelectionWithoutEdgesOnSwitchOrStart() {
        RobotProfile profile = RobotProfile.load(new java.io.File("../robot-configs"),
                "biobuzz-competitionbot", "competitionbot");
        try (MockedStatic<RobotProfile> profiles = mockStatic(RobotProfile.class);
                MockedConstruction<GateSubsystem> gates = mockConstruction(GateSubsystem.class);
                MockedConstruction<DriveSubsystem> drives = mockConstruction(DriveSubsystem.class);
                MockedConstruction<IntakeSubsystem> intakes = mockConstruction(IntakeSubsystem.class);
                MockedConstruction<LauncherSubsystem> launchers = mockConstruction(LauncherSubsystem.class);
                MockedConstruction<VisionSubsystem> visions = mockConstruction(VisionSubsystem.class);
                MockedConstruction<DriveModePreferences> prefs = mockConstruction(DriveModePreferences.class);
                MockedConstruction<MechanismCoordinator> mechanisms = mockConstruction(MechanismCoordinator.class);
                MockedConstruction<TeleopController> controllers = mockConstruction(TeleopController.class);
                MockedConstruction<StarterBotTelemetry> displays = mockConstruction(StarterBotTelemetry.class)) {
            profiles.when(() -> RobotProfile.loadActive("competitionbot")).thenReturn(profile);
            CompetitionBotTeleOp op = new CompetitionBotTeleOp();
            op.hardwareMap = mock(HardwareMap.class);
            op.gamepad1 = mock(Gamepad.class);
            op.gamepad2 = mock(Gamepad.class);
            op.telemetry = mock(Telemetry.class);
            op.init();
            GateSubsystem gate = gates.constructed().get(0);
            op.gamepad2.b = true;
            op.gamepad1.dpad_up = true;
            op.init_loop();
            op.gamepad1.dpad_up = false;
            op.init_loop();
            op.gamepad1.dpad_right = true;
            op.init_loop();
            verify(gate, never()).update(anyBoolean());
            op.gamepad2.b = false;
            op.init_loop();
            op.gamepad2.b = true;
            op.init_loop();
            verify(gate).update(true);
            op.start();
            verify(controllers.constructed().get(0)).configure(any(), eq(op.gamepad1), eq(op.gamepad2));
            op.loop();
            verify(gate, never()).update(false);
            op.gamepad1.b = true;
            op.gamepad2.b = false;
            op.loop();
            op.gamepad2.b = true;
            op.loop();
            verify(gate).update(false);
            verify(controllers.constructed().get(0), times(3)).update(op.gamepad1, op.gamepad2);
            op.stop();
        }
    }

    @Test
    public void stopAfterPartialInitializationStillStopsInitializedHardware() {
        RobotProfile profile = RobotProfile.load(new java.io.File("../robot-configs"),
                "biobuzz-competitionbot", "competitionbot");
        try (MockedStatic<RobotProfile> profiles = mockStatic(RobotProfile.class);
                MockedConstruction<GateSubsystem> gates = mockConstruction(GateSubsystem.class);
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
            verify(gates.constructed().get(0)).update(false);
            verify(drives.constructed().get(0)).driveRobotRelative(0, 0, 0);
            verify(intakes.constructed().get(0)).setPower(0);
            verify(visions.constructed().get(0)).stop();
        }
    }
}
