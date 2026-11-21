package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.io.File;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control.DriveModePreferences;
import org.firstinspires.ftc.teamcode.subsystems.ConfigurationSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FieldMap;
import org.firstinspires.ftc.teamcode.subsystems.RobotProfile;
import org.firstinspires.ftc.teamcode.subsystems.TrackingSubsystem;
import org.junit.Test;
import org.junit.Rule;
import org.junit.rules.TemporaryFolder;
import org.mockito.MockedConstruction;
import org.mockito.MockedStatic;

public class SimpleLeaveAndParkAutoTest {
    @Rule public TemporaryFolder temporary = new TemporaryFolder();

    @Test public void movesExactlyOneProfileChassisLengthWithoutConfiguration() {
        assertEquals("Simple Leave & Park",
                SimpleLeaveAndParkAuto.class.getAnnotation(Autonomous.class).name());
        for (String robot : new String[] {"starterbot", "competitionbot"}) {
            RobotProfile profile = RobotProfile.load(new File("../robot-configs"),
                    "biobuzz-" + robot, robot);
            try (MockedStatic<RobotProfile> profiles = mockStatic(RobotProfile.class);
                 MockedConstruction<DriveModePreferences> prefs = mockConstruction(DriveModePreferences.class);
                 MockedConstruction<ConfigurationSubsystem> configurations = mockConstruction(ConfigurationSubsystem.class);
                 MockedConstruction<DriveSubsystem> drives = mockConstruction(DriveSubsystem.class);
                 MockedConstruction<TrackingSubsystem> trackers = mockConstruction(TrackingSubsystem.class)) {
                profiles.when(() -> RobotProfile.loadActive(null)).thenReturn(profile);
                SimpleLeaveAndParkAuto op = createOp();
                when(op.getRuntime()).thenReturn(0.0, 0.1, 0.2, 0.3);
                op.init();
                DriveSubsystem drive = drives.constructed().get(0);
                TrackingSubsystem tracking = trackers.constructed().get(0);
                double distance = profile.chassisLengthCm / 2.54;
                op.gamepad1.dpad_right = true;
                op.init_loop();
                assertTrue(prefs.constructed().isEmpty());
                assertTrue(configurations.constructed().isEmpty());
                verify(op.telemetry, atLeastOnce()).addData("After START",
                        "Moves forward one chassis length: %.1f in (%.1f cm)",
                        distance, profile.chassisLengthCm);
                verify(drive, never()).driveRobotRelative(doubleThat(p -> p != 0), anyDouble(), anyDouble());
                when(tracking.hasValidPose()).thenReturn(true);
                when(tracking.getPose()).thenReturn(new FieldMap.Pose(0, 0, 90));
                op.start();
                verify(tracking).start(argThat(p -> p.x == 0 && p.y == 0 && p.headingDegrees == 90));
                verify(drive, never()).driveRobotRelative(doubleThat(p -> p != 0), anyDouble(), anyDouble());
                clearInvocations(drive);
                op.gamepad1.b = true;
                op.gamepad1.y = true;
                when(tracking.getPose()).thenReturn(new FieldMap.Pose(0, distance - 0.01, 90));
                op.loop();
                verify(drive).driveRobotRelative(profile.navigation.finalApproachPower, 0, 0);
                clearInvocations(drive);
                when(tracking.getPose()).thenReturn(new FieldMap.Pose(0, distance, 90));
                op.loop();
                verify(drive).driveRobotRelative(0, 0, 0);
                verify(op.telemetry).addLine("Complete: moved forward one chassis length");
                clearInvocations(drive);
                op.loop();
                verify(drive).driveRobotRelative(0, 0, 0);
                verify(drive, never()).driveRobotRelative(doubleThat(p -> p != 0), anyDouble(), anyDouble());
            }
        }
    }

    @Test public void failsSafeOnTrackingDriftTimeoutOrDriverStop() {
        for (String scenario : new String[] {"tracking", "heading", "sideways", "progress", "timeout", "stop"}) {
            RobotProfile profile = TestProfiles.starter();
            try (MockedStatic<RobotProfile> profiles = mockStatic(RobotProfile.class);
                 MockedConstruction<DriveSubsystem> drives = mockConstruction(DriveSubsystem.class);
                 MockedConstruction<TrackingSubsystem> trackers = mockConstruction(TrackingSubsystem.class)) {
                profiles.when(() -> RobotProfile.loadActive(null)).thenReturn(profile);
                SimpleLeaveAndParkAuto op = createOp();
                when(op.getRuntime()).thenReturn(0.0, scenario.equals("timeout") ? profile.navigation.timeoutSeconds + 0.1
                        : scenario.equals("progress") ? 4.1 : 0.1);
                op.init();
                TrackingSubsystem tracking = trackers.constructed().get(0);
                DriveSubsystem drive = drives.constructed().get(0);
                when(tracking.hasValidPose()).thenReturn(!scenario.equals("tracking"));
                when(tracking.getPose()).thenReturn(new FieldMap.Pose(
                        scenario.equals("sideways") ? 1.1 : 0,
                        0, scenario.equals("heading") ? 94 : 90));
                op.start();
                clearInvocations(drive);
                if (scenario.equals("stop")) op.stop(); else op.loop();
                verify(drive).driveRobotRelative(0, 0, 0);
                verify(drive, never()).driveRobotRelative(doubleThat(p -> p != 0), anyDouble(), anyDouble());
                op.gamepad1.y = false;
                op.gamepad1.b = false;
                clearInvocations(drive);
                op.loop();
                verify(drive).driveRobotRelative(0, 0, 0);
                verify(drive, never()).driveRobotRelative(doubleThat(p -> p != 0), anyDouble(), anyDouble());
            }
        }
    }

    @Test public void usesProfilePowerDriftAndTimeoutInsteadOfFormerConstants() throws Exception {
        String json = new String(Files.readAllBytes(
                new File("../robot-configs/biobuzz-starterbot.profile.json").toPath()),
                StandardCharsets.UTF_8);
        File profileFile = temporary.newFile("biobuzz-starterbot.profile.json");
        com.google.gson.JsonObject root = new com.google.gson.Gson()
                .fromJson(json, com.google.gson.JsonObject.class);
        com.google.gson.JsonObject navigation = root.getAsJsonObject("navigation");
        navigation.addProperty("finalApproachPower", 0.12);
        navigation.addProperty("headingToleranceDegrees", 6.0);
        navigation.addProperty("arrivalToleranceIn", 2.0);
        navigation.addProperty("timeoutSeconds", 2.0);
        Files.write(profileFile.toPath(), root.toString().getBytes(StandardCharsets.UTF_8));
        RobotProfile profile = RobotProfile.load(temporary.getRoot(), "biobuzz-starterbot", "starterbot");
        try (MockedStatic<RobotProfile> profiles = mockStatic(RobotProfile.class);
             MockedConstruction<DriveSubsystem> drives = mockConstruction(DriveSubsystem.class);
             MockedConstruction<TrackingSubsystem> trackers = mockConstruction(TrackingSubsystem.class)) {
            profiles.when(() -> RobotProfile.loadActive(null)).thenReturn(profile);
            SimpleLeaveAndParkAuto op = createOp();
            when(op.getRuntime()).thenReturn(0.0, 0.1, 2.0);
            op.init();
            TrackingSubsystem tracking = trackers.constructed().get(0);
            when(tracking.hasValidPose()).thenReturn(true);
            when(tracking.getPose()).thenReturn(new FieldMap.Pose(1.5, 1, 95));
            op.start();
            DriveSubsystem drive = drives.constructed().get(0);
            clearInvocations(drive);
            op.loop();
            verify(drive).driveRobotRelative(0.12, 0, 0);
            clearInvocations(drive);
            op.loop();
            verify(drive).driveRobotRelative(0, 0, 0);
            verify(op.telemetry).addLine("Stopped: timeout");
        }
    }

    private SimpleLeaveAndParkAuto createOp() {
        SimpleLeaveAndParkAuto op = spy(new SimpleLeaveAndParkAuto());
        op.hardwareMap = mock(HardwareMap.class);
        op.gamepad1 = new Gamepad();
        op.telemetry = mock(Telemetry.class);
        return op;
    }

    @Test public void stopBeforeInitIsSafe() {
        new SimpleLeaveAndParkAuto().stop();
    }
}
