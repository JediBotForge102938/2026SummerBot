package org.firstinspires.ftc.teamcode.telemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

/** Aligned rows for the Driver Station's limited HTML renderer. */
public final class TelemetryTable {
    private TelemetryTable() { }

    public static void heading(Telemetry telemetry, String title) {
        telemetry.addLine("<b>" + escape(title) + "</b>");
    }

    public static void row(Telemetry telemetry, String label, String value,
            boolean selected, RobotTelemetry.Color color) {
        writeRow(telemetry, label, value, selected, color == null ? null
                : color == RobotTelemetry.Color.RED ? "#FF6666" : "#66AAFF");
    }

    public static void statusRow(Telemetry telemetry, String label, String value,
            RobotTelemetry.Level level) {
        String color;
        switch (level) {
            case SUCCESS:
                color = "#66FF99";
                break;
            case WARNING:
                color = "#FFCC66";
                break;
            case ERROR:
                color = "#FF6666";
                break;
            default:
                color = null;
        }
        writeRow(telemetry, label, value, false, color);
    }

    private static void writeRow(Telemetry telemetry, String label, String value,
            boolean selected, String color) {
        String marker = selected ? "&gt;&nbsp;" : "&nbsp;&nbsp;";
        String key = escape(String.format(Locale.US, "%-15s", label + ":"))
                .replace(" ", "&nbsp;");
        String text = escape(value);
        if (color != null) {
            text = "<font color='" + color + "'>" + text + "</font>";
        }
        telemetry.addLine("<tt>" + marker + key + text + "</tt>");
    }

    public static void row(Telemetry telemetry, String label, String value) {
        row(telemetry, label, value, false, null);
    }

    private static String escape(String text) {
        return text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
                .replace("\"", "&quot;").replace("'", "&#39;");
    }
}
