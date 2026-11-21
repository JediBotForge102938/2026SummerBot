package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.*;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control.DriveModePreferences;
import org.firstinspires.ftc.teamcode.control.LandmarkNavigationService;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotProfile;
import org.firstinspires.ftc.teamcode.subsystems.TrackingSubsystem;
import org.junit.Test;
import org.mockito.MockedConstruction;
import org.mockito.MockedStatic;

public class LeaveAndParkAutoTest {
    @Test public void registeredAsAutonomous() {
        assertEquals("Leave & Park", LeaveAndParkAuto.class.getAnnotation(Autonomous.class).name());
    }

    @Test public void initSelectionsDetermineStartAndParkingWithoutAOrManualDriving() {
        for (boolean red : new boolean[] {true, false}) {
            for (boolean scoring : new boolean[] {false, true}) {
                assertRoute(red, scoring);
            }
        }
    }

    private void assertRoute(boolean red, boolean scoring) {
        RobotProfile profile = TestProfiles.starter();
        try (MockedStatic<RobotProfile> profiles = mockStatic(RobotProfile.class);
             MockedConstruction<DriveSubsystem> drives = mockConstruction(DriveSubsystem.class);
             MockedConstruction<TrackingSubsystem> trackers = mockConstruction(TrackingSubsystem.class);
             MockedConstruction<DriveModePreferences> prefs = mockConstruction(DriveModePreferences.class,
                     (mock, context) -> {
                         when(mock.getConfigurationRedAlliance()).thenReturn(red);
                         when(mock.isScoringStart()).thenReturn(scoring);
                     });
             MockedConstruction<LandmarkNavigationService> navigators =
                     mockConstruction(LandmarkNavigationService.class)) {
            profiles.when(() -> RobotProfile.loadActive(null)).thenReturn(profile);
            LeaveAndParkAuto op = new LeaveAndParkAuto();
            op.hardwareMap = mock(HardwareMap.class);
            op.gamepad1 = mock(Gamepad.class);
            op.telemetry = mock(Telemetry.class);
            op.init();
            op.init_loop();
            LandmarkNavigationService navigation = navigators.constructed().get(0);
            verify(navigation, never()).start(any(), anyDouble());
            op.start();
            verify(trackers.constructed().get(0)).start(argThat(pose ->
                    pose.x == (red ? -12 : 12) && pose.y == (scoring ? 63 : -63)
                            && pose.headingDegrees == (scoring ? -90 : 90)));
            verify(navigation).start(argThat(target ->
                    target.name.equals((red ? "Red parking" : "Blue parking")
                            + (scoring ? " scoring edge" : " audience edge"))
                            && target.robotPose.x == (red ? -60 : 60)
                            && target.robotPose.y == (red ? scoring ? 53 : 17
                            : scoring ? -18 : -54)
                            && target.robotPose.headingDegrees == (red ? 0 : 180)), anyDouble());
            when(navigation.isActive()).thenReturn(true);
            op.gamepad1.left_stick_y = -1;
            op.loop();
            verify(navigation).update(anyDouble());
            op.gamepad1.y = true;
            op.loop();
            verify(navigation).cancel("Stopped by Y");
            when(navigation.isActive()).thenReturn(false);
            op.gamepad1.y = false;
            op.loop();
            verify(navigation, times(1)).start(any(), anyDouble());
            verify(drives.constructed().get(0), never())
                    .driveRobotRelative(anyDouble(), anyDouble(), anyDouble(), anyDouble());
            op.stop();
            verify(navigation).cancel("OpMode stopped");
            verify(drives.constructed().get(0), atLeastOnce()).driveRobotRelative(0, 0, 0);
        }
    }

    @Test public void stopIsSafeBeforeInitialization() {
        new LeaveAndParkAuto().stop();
    }
}
