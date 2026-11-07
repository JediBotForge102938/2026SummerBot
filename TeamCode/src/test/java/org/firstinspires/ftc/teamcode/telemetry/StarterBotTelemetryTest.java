package org.firstinspires.ftc.teamcode.telemetry;

import static org.junit.Assert.assertThrows;
import static org.mockito.Mockito.*;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control.HiveTrackingController;
import org.firstinspires.ftc.teamcode.control.TeleopController;
import org.firstinspires.ftc.teamcode.control.DriveModePreferences;
import org.firstinspires.ftc.teamcode.subsystems.ConfigurationSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.junit.Before;
import org.junit.Test;

public class StarterBotTelemetryTest {
    private Telemetry telemetry;
    private TeleopController controller;
    private HiveTrackingController tracking;
    private LauncherSubsystem launcher;
    private ConfigurationSubsystem configuration;
    private StarterBotTelemetry display;

    @Before
    public void setUp() {
        telemetry = mock(Telemetry.class);
        configuration = new ConfigurationSubsystem();
        DriveModePreferences preferences = mock(DriveModePreferences.class);
        when(preferences.getConfigurationRedAlliance()).thenReturn(true);
        when(preferences.getConfigurationFieldCentric()).thenReturn(true);
        configuration.init(telemetry, mock(Gamepad.class), preferences);
        configuration.lock();
        controller = mock(TeleopController.class);
        tracking = mock(HiveTrackingController.class);
        launcher = mock(LauncherSubsystem.class);
        when(controller.isFieldCentric()).thenReturn(true);
        when(tracking.getTrackingStatus()).thenReturn("MANUAL");
        when(tracking.getTrackingStatusLevel()).thenReturn(RobotTelemetry.Level.INFO);
        when(tracking.getTargetName()).thenReturn("None");
        when(launcher.getVelocity()).thenReturn(1234.0);
        when(launcher.getTargetVelocity()).thenReturn(1250);
        when(launcher.getStatus()).thenReturn(LauncherSubsystem.Status.READY);
        display = new StarterBotTelemetry(telemetry, configuration, controller, tracking, launcher);
        clearInvocations(telemetry);
    }

    @Test
    public void compactFrameContainsSpacedConfigurationTrackingAndLauncherTables() {
        display.publish();
        verify(telemetry).clearAll();
        verify(telemetry).addLine("<b>Configuration</b>");
        verify(telemetry).addLine("<tt>&nbsp;&nbsp;Alliance:&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;"
                + "<font color='#FF6666'>Red</font></tt>");
        verify(telemetry).addLine("<tt>&nbsp;&nbsp;Drive&nbsp;mode:&nbsp;&nbsp;&nbsp;&nbsp;"
                + "Field-centric</tt>");
        verify(telemetry).addLine("<tt>&nbsp;&nbsp;Starting&nbsp;spot:&nbsp;Audience</tt>");
        verify(telemetry).addLine("<b>Hive Tracking</b>");
        verify(telemetry).addLine("<tt>&nbsp;&nbsp;Status:&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;"
                + "&nbsp;&nbsp;MANUAL</tt>");
        verify(telemetry).addLine("<tt>&nbsp;&nbsp;Target:&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;"
                + "&nbsp;&nbsp;None</tt>");
        verify(telemetry, times(2)).addLine("&nbsp;");
        verify(telemetry).addLine("<b>Launcher</b>");
        verify(telemetry).addLine("<tt>&nbsp;&nbsp;Status:&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;"
                + "&nbsp;&nbsp;<font color='#66FF99'>READY</font></tt>");
        verify(telemetry).addLine("<tt>&nbsp;&nbsp;Actual/target:&nbsp;"
                + "1234 / 1250 ticks/s</tt>");
        verify(telemetry).update();
        verifyNoMoreInteractions(telemetry);
        verify(tracking, never()).getTargetRangeInches();
    }

    @Test
    public void offLauncherStatusIsSeparateFromActualAndTargetVelocity() {
        when(launcher.getStatus()).thenReturn(LauncherSubsystem.Status.OFF);
        when(launcher.getVelocity()).thenReturn(0.0);
        display.publish();
        verify(telemetry).addLine("<tt>&nbsp;&nbsp;Status:&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;"
                + "&nbsp;&nbsp;OFF</tt>");
        verify(telemetry).addLine("<tt>&nbsp;&nbsp;Actual/target:&nbsp;"
                + "0 / 1250 ticks/s</tt>");
    }

    @Test
    public void liveModeAndTargetDetailsUpdateWithoutRetainingOldRows() {
        when(controller.isFieldCentric()).thenReturn(false);
        when(tracking.hasTarget()).thenReturn(true);
        when(tracking.getTrackingStatus()).thenReturn("LOCKED IN");
        when(tracking.getTrackingStatusLevel()).thenReturn(RobotTelemetry.Level.SUCCESS);
        when(tracking.getTargetName()).thenReturn("RED SCORING");
        when(tracking.getTargetRangeInches()).thenReturn(43.25);
        when(tracking.getHorizontalErrorDegrees()).thenReturn(-2.5);
        display.publish();
        verify(telemetry).addLine("<tt>&nbsp;&nbsp;Drive&nbsp;mode:&nbsp;&nbsp;&nbsp;&nbsp;"
                + "Robot-centric</tt>");
        verify(telemetry).addLine("<tt>&nbsp;&nbsp;Range&nbsp;/&nbsp;error:&nbsp;"
                + "43.3 in / -2.5 deg</tt>");
        verify(telemetry).addLine("<tt>&nbsp;&nbsp;Target:&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;"
                + "&nbsp;&nbsp;<font color='#66FF99'>RED SCORING</font></tt>");
        clearInvocations(telemetry);
        when(tracking.hasTarget()).thenReturn(false);
        when(tracking.getTargetName()).thenReturn("None");
        when(tracking.getTrackingStatus()).thenReturn("SEARCHING");
        when(tracking.getTrackingStatusLevel()).thenReturn(RobotTelemetry.Level.WARNING);
        display.publish();
        verify(telemetry).clearAll();
        verify(telemetry, never()).addLine(contains("Range&nbsp;/&nbsp;error"));
        verify(telemetry).addLine(contains("<font color='#FFCC66'>SEARCHING</font>"));
        verify(telemetry).addLine("<tt>&nbsp;&nbsp;Target:&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;"
                + "&nbsp;&nbsp;None</tt>");
    }

    @Test
    public void hiveStatusesUseGreenRedAndYellow() {
        String[] statuses = {"LOCKED IN", "NOT SCORABLE", "SEARCHING"};
        RobotTelemetry.Level[] levels = {RobotTelemetry.Level.SUCCESS,
                RobotTelemetry.Level.ERROR, RobotTelemetry.Level.WARNING};
        String[] colors = {"#66FF99", "#FF6666", "#FFCC66"};
        for (int index = 0; index < statuses.length; index++) {
            when(tracking.getTrackingStatus()).thenReturn(statuses[index]);
            when(tracking.getTrackingStatusLevel()).thenReturn(levels[index]);
            display.publish();
            verify(telemetry).addLine("<tt>&nbsp;&nbsp;Status:&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;"
                    + "&nbsp;&nbsp;<font color='" + colors[index] + "'>"
                    + statuses[index] + "</font></tt>");
        }
    }

    @Test
    public void competitionShowsDisabledFeedGateWithoutChangingLauncherStatus() {
        display = new StarterBotTelemetry(telemetry, configuration, controller, tracking, launcher,
                false);
        display.publish();
        verify(telemetry).addLine("<tt>&nbsp;&nbsp;Feed&nbsp;gate:&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;"
                + "<font color='#FFCC66'>DISABLED: positions not configured</font></tt>");
        verify(telemetry).addLine(contains("<font color='#66FF99'>READY</font>"));
    }

    @Test
    public void launcherStatusIsYellowUntilReadyAndAgainWhenSpeedDrops() {
        for (LauncherSubsystem.Status status : new LauncherSubsystem.Status[] {
                LauncherSubsystem.Status.SPINNING_UP, LauncherSubsystem.Status.BELOW_TARGET}) {
            when(launcher.getStatus()).thenReturn(status);
            display.publish();
            verify(telemetry).addLine("<tt>&nbsp;&nbsp;Status:&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;"
                    + "&nbsp;&nbsp;<font color='#FFCC66'>"
                    + status.name().replace('_', ' ') + "</font></tt>");
        }
    }

    @Test
    public void invalidLauncherReadingsAreNotReportedAsSuccessfulTelemetry() {
        when(launcher.getVelocity()).thenThrow(new IllegalStateException("Invalid launcher velocity"));
        assertThrows(IllegalStateException.class, () -> display.publish());
        verify(telemetry, never()).update();
    }

    @Test
    public void tableEscapesValuesInsteadOfInterpretingThemAsHtml() {
        TelemetryTable.row(telemetry, "Target", "<unknown>&");
        verify(telemetry).addLine("<tt>&nbsp;&nbsp;Target:&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;"
                + "&nbsp;&nbsp;&lt;unknown&gt;&amp;</tt>");
    }
}
