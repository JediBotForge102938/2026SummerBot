package org.firstinspires.ftc.teamcode.telemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control.HiveTrackingController;
import org.firstinspires.ftc.teamcode.control.TeleopController;
import org.firstinspires.ftc.teamcode.subsystems.ConfigurationSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;

import java.util.Locale;

public class StarterBotTelemetry {
    private final Telemetry telemetry;
    private final ConfigurationSubsystem configuration;
    private final TeleopController controller;
    private final HiveTrackingController tracking;
    private final LauncherSubsystem launcher;
    private final boolean feederEnabled;

    public StarterBotTelemetry(Telemetry telemetry, ConfigurationSubsystem configuration,
            TeleopController controller, HiveTrackingController tracking, LauncherSubsystem launcher) {
        this(telemetry, configuration, controller, tracking, launcher, true);
    }

    public StarterBotTelemetry(Telemetry telemetry, ConfigurationSubsystem configuration,
            TeleopController controller, HiveTrackingController tracking, LauncherSubsystem launcher,
            boolean feederEnabled) {
        this.telemetry = telemetry;
        this.configuration = configuration;
        this.controller = controller;
        this.tracking = tracking;
        this.launcher = launcher;
        this.feederEnabled = feederEnabled;
    }

    public void publish() {
        telemetry.clearAll();
        configuration.publish(controller.isFieldCentric());
        telemetry.addLine("&nbsp;");
        TelemetryTable.heading(telemetry, "Hive Tracking");
        TelemetryTable.statusRow(telemetry, "Status", tracking.getTrackingStatus(),
                tracking.getTrackingStatusLevel());
        boolean hasTarget = tracking.hasTarget();
        TelemetryTable.statusRow(telemetry, "Target", tracking.getTargetName(),
                hasTarget ? RobotTelemetry.Level.SUCCESS : RobotTelemetry.Level.INFO);
        if (hasTarget) {
            TelemetryTable.row(telemetry, "Range / error", String.format(Locale.US,
                    "%.1f in / %.1f deg", tracking.getTargetRangeInches(),
                    tracking.getHorizontalErrorDegrees()));
        }
        telemetry.addLine("&nbsp;");
        TelemetryTable.heading(telemetry, "Launcher");
        LauncherSubsystem.Status launcherStatus = launcher.getStatus();
        TelemetryTable.statusRow(telemetry, "Status", launcherStatus.name().replace('_', ' '),
                launcherStatus == LauncherSubsystem.Status.READY ? RobotTelemetry.Level.SUCCESS
                        : launcherStatus == LauncherSubsystem.Status.OFF ? RobotTelemetry.Level.INFO
                        : RobotTelemetry.Level.WARNING);
        TelemetryTable.row(telemetry, "Actual/target", String.format(Locale.US,
                "%.0f / %d ticks/s", launcher.getVelocity(), launcher.getTargetVelocity()));
        if (!feederEnabled) {
            TelemetryTable.statusRow(telemetry, "Feed gate", "DISABLED: positions not configured",
                    RobotTelemetry.Level.WARNING);
        }
        telemetry.update();
    }
}
