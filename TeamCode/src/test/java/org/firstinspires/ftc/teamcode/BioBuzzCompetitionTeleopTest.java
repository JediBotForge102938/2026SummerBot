package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control.DriveModePreferences;
import org.firstinspires.ftc.teamcode.control.TeleopController;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.telemetry.DriverStationTelemetry;
import org.firstinspires.ftc.teamcode.telemetry.TelemetryPresenter;
import org.junit.Test;
import org.mockito.MockedConstruction;

public class BioBuzzCompetitionTeleopTest {
    @Test
    public void competitionUsesSharedControllerForInitLoopDriveLoopAndStop() {
        try (MockedConstruction<DriveSubsystem> drives = mockConstruction(DriveSubsystem.class);
                MockedConstruction<IntakeSubsystem> intakes = mockConstruction(IntakeSubsystem.class,
                        (mock, context) -> assertEquals(false, context.arguments().get(0)));
                MockedConstruction<LauncherSubsystem> launchers = mockConstruction(LauncherSubsystem.class);
                MockedConstruction<VisionSubsystem> visions = mockConstruction(VisionSubsystem.class);
                MockedConstruction<DriveModePreferences> preferences =
                        mockConstruction(DriveModePreferences.class);
                MockedConstruction<DriverStationTelemetry> outputs =
                        mockConstruction(DriverStationTelemetry.class);
                MockedConstruction<TeleopController> controllers =
                        mockConstruction(TeleopController.class);
                MockedConstruction<TelemetryPresenter> displays =
                        mockConstruction(TelemetryPresenter.class)) {
            BioBuzzCompetitionbotTeleopMecanumFieldCentric opMode =
                    new BioBuzzCompetitionbotTeleopMecanumFieldCentric();
            opMode.hardwareMap = mock(HardwareMap.class);
            opMode.gamepad1 = mock(Gamepad.class);
            opMode.telemetry = mock(Telemetry.class);
            opMode.init();

            TeleopController controller = controllers.constructed().get(0);
            verify(drives.constructed().get(0)).init(opMode.hardwareMap);
            verify(intakes.constructed().get(0)).init(opMode.hardwareMap);
            verify(launchers.constructed().get(0)).init(opMode.hardwareMap);
            verify(visions.constructed().get(0)).init(opMode.hardwareMap);
            verify(controller).updateInit(opMode.gamepad1);
            opMode.init_loop();
            verify(controller, times(2)).updateInit(opMode.gamepad1);
            opMode.loop();
            verify(controller).update(opMode.gamepad1);
            verify(drives.constructed().get(0), never()).resetHeading();
            opMode.stop();
            verify(controller).stop();
            verify(opMode.gamepad1).stopRumble();
            verify(visions.constructed().get(0)).stop();
            verify(displays.constructed().get(0), times(4)).publish();
        }
    }
}
