package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertThrows;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.*;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.mockito.MockedConstruction;
import org.firstinspires.ftc.teamcode.control.DriveModePreferences;
import org.firstinspires.ftc.teamcode.subsystems.RobotProfile;
import org.mockito.MockedStatic;

public class ConfigurationTestTeleOpTest {
    private ConfigurationTestTeleOp opMode;
    private IMU imu;
    private MockedConstruction<RevHubOrientationOnRobot> orientation;
    private MockedConstruction<DriveModePreferences> preferences;
    private MockedStatic<RobotProfile> profiles;

    @Before
    public void setUp() {
        orientation = mockConstruction(RevHubOrientationOnRobot.class);
        RobotProfile profile = TestProfiles.starter();
        profiles = mockStatic(RobotProfile.class);
        profiles.when(() -> RobotProfile.loadActive(null)).thenReturn(profile);
        preferences = mockConstruction(DriveModePreferences.class, (mock, context) -> {
            when(mock.getConfigurationRedAlliance()).thenReturn(true);
            when(mock.getConfigurationFieldCentric()).thenReturn(true);
        });
        opMode = new ConfigurationTestTeleOp();
        opMode.hardwareMap = mock(HardwareMap.class);
        opMode.gamepad1 = mock(Gamepad.class);
        opMode.telemetry = mock(Telemetry.class);
        imu = mock(IMU.class);
        when(opMode.hardwareMap.get(IMU.class, "imu")).thenReturn(imu);
        when(imu.initialize(any(IMU.Parameters.class))).thenReturn(true);
        yaw(0);
    }

    @After
    public void tearDown() {
        orientation.close();
        preferences.close();
        profiles.close();
    }

    @Test
    public void initShowsOnlyConfigurationAndAlignedSelectionsWithoutResettingYawOrMappingMotors() {
        opMode.init();
        clearInvocations(opMode.telemetry);
        opMode.init_loop();
        verify(opMode.telemetry).clearAll();
        verify(opMode.telemetry).addLine("<b>Configuration</b>");
        verify(opMode.telemetry).addLine(
                "<tt>&nbsp;&nbsp;Robot&nbsp;profile:&nbsp;biobuzz-starterbot</tt>");
        verify(opMode.telemetry).addLine(allianceRow(true, true));
        verify(opMode.telemetry).addLine(driveRow(false, "Field-centric"));
        verify(opMode.telemetry).addLine(spotRow(false, "Audience"));
        verify(opMode.telemetry).update();
        verifyNoMoreInteractions(opMode.telemetry);
        verify(opMode.gamepad1).setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
        verify(imu, never()).resetYaw();
        verify(opMode.hardwareMap).get(IMU.class, "imu");
        verifyNoMoreInteractions(opMode.hardwareMap);
    }

    @Test
    public void choicesChangeOncePerPressAndAllThreeRowsAreSelectable() {
        opMode.init();
        opMode.gamepad1.dpad_right = true;
        opMode.init_loop();
        opMode.init_loop();
        verify(opMode.telemetry, times(2)).addLine(allianceRow(true, false));
        release();
        down();
        right();
        down();
        right();
        clearInvocations(opMode.telemetry);
        opMode.start();
        verify(opMode.telemetry).addLine(allianceRow(false, false));
        verify(opMode.telemetry).addLine(driveRow(false, "Robot-centric"));
        verify(opMode.telemetry).addLine(spotRow(false, "Scoring"));
        verify(opMode.telemetry).addData("Field heading (deg)", "%.2f", 270.0);
        verify(opMode.gamepad1).setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
    }

    @Test
    public void upWrapsToStartingSpotAndLeftChangesItsValue() {
        opMode.init();
        opMode.gamepad1.dpad_up = true;
        opMode.init_loop();
        opMode.init_loop();
        release();
        opMode.gamepad1.dpad_left = true;
        opMode.init_loop();
        verify(opMode.telemetry).addLine(spotRow(true, "Scoring"));
    }

    @Test
    public void opposingDirectionsDoNotChangeRowOrValue() {
        opMode.init();
        opMode.gamepad1.dpad_up = true;
        opMode.gamepad1.dpad_down = true;
        opMode.gamepad1.dpad_left = true;
        opMode.gamepad1.dpad_right = true;
        opMode.init_loop();
        verify(opMode.telemetry, times(2)).addLine(allianceRow(true, true));
        verify(opMode.telemetry, never()).addLine(spotRow(true, "Audience"));
    }

    @Test
    public void startResetsOnceLocksSelectionAndBlueHeadingWraps() {
        opMode.init();
        right();
        opMode.start();
        verify(opMode.telemetry).addData("Field heading (deg)", "%.2f", 90.0);
        yaw(190);
        opMode.gamepad1.dpad_left = true;
        opMode.loop();
        opMode.init_loop();
        opMode.start();
        verify(imu).resetYaw();
        verify(opMode.gamepad1).setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
        verify(opMode.gamepad1).setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
        verifyNoMoreInteractions(opMode.gamepad1);
        verify(opMode.telemetry, atLeastOnce()).addLine(allianceRow(false, false));
        verify(opMode.telemetry, never()).addLine(allianceRow(false, true));
        verify(opMode.telemetry, atLeastOnce())
                .addData("Field heading (deg)", "%.2f", 280.0);
    }

    @Test
    public void redHeadingWrapsNegativeYawAndChoicesResetOnNewInit() {
        opMode.init();
        opMode.start();
        verify(opMode.telemetry).addData("Field heading (deg)", "%.2f", 90.0);
        yaw(-100);
        opMode.loop();
        verify(opMode.telemetry).addData("Field heading (deg)", "%.2f", 350.0);
        opMode.init();
        right();
        down();
        right();
        down();
        right();
        clearInvocations(opMode.telemetry);
        opMode.init();
        verify(opMode.telemetry).addLine(allianceRow(true, true));
        verify(opMode.telemetry).addLine(driveRow(false, "Field-centric"));
        verify(opMode.telemetry).addLine(spotRow(false, "Audience"));
        verify(imu).resetYaw();
    }

    @Test
    public void failedInitializationAndInvalidYawAreExplicitErrors() {
        when(imu.initialize(any(IMU.Parameters.class))).thenReturn(false);
        assertThrows(IllegalStateException.class, () -> opMode.init());
        verify(imu, never()).resetYaw();
        when(imu.initialize(any(IMU.Parameters.class))).thenReturn(true);
        yaw(Double.NaN);
        opMode.init();
        opMode.start();
        verify(opMode.telemetry, atLeastOnce()).addLine(
                "<b><font color='#FF6666'>IMU ERROR: invalid yaw</font></b>");
        verify(opMode.telemetry, never()).addData(eq("Field heading (deg)"), anyString(),
                any(Object[].class));
    }

    private static String allianceRow(boolean selected, boolean red) {
        return "<tt>" + marker(selected) + "Alliance:&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;"
                + "<font color='" + (red ? "#FF6666" : "#66AAFF") + "'>"
                + (red ? "Red" : "Blue") + "</font></tt>";
    }

    private static String driveRow(boolean selected, String value) {
        return "<tt>" + marker(selected) + "Drive&nbsp;mode:&nbsp;&nbsp;&nbsp;&nbsp;"
                + value + "</tt>";
    }

    private static String spotRow(boolean selected, String value) {
        return "<tt>" + marker(selected) + "Starting&nbsp;spot:&nbsp;" + value + "</tt>";
    }

    private static String marker(boolean selected) {
        return selected ? "&gt;&nbsp;" : "&nbsp;&nbsp;";
    }

    private void yaw(double degrees) {
        when(imu.getRobotYawPitchRollAngles()).thenReturn(
                new YawPitchRollAngles(AngleUnit.DEGREES, degrees, 0, 0, 0));
    }

    private void release() {
        opMode.gamepad1.dpad_up = false;
        opMode.gamepad1.dpad_down = false;
        opMode.gamepad1.dpad_left = false;
        opMode.gamepad1.dpad_right = false;
        opMode.init_loop();
    }

    private void down() {
        opMode.gamepad1.dpad_down = true;
        opMode.init_loop();
        release();
    }

    private void right() {
        opMode.gamepad1.dpad_right = true;
        opMode.init_loop();
        release();
    }
}
