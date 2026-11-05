package org.firstinspires.ftc.teamcode.telemetry;

import android.content.Context;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;
import java.util.Objects;

/** Owns the SDK telemetry display; do not mix direct SDK writes into its frames. */
public class DriverStationTelemetry implements RobotTelemetry {
    private final Telemetry telemetry;
    private final boolean styled;

    public DriverStationTelemetry(Telemetry telemetry) {
        this(telemetry, true);
    }

    public DriverStationTelemetry(Telemetry telemetry, Context context) {
        // The bundled simulator identifies its Context this way and ignores HTML display format.
        this(telemetry, !"virtual_robot".equals(context.getPackageName()));
    }

    public DriverStationTelemetry(Telemetry telemetry, boolean styled) {
        this.telemetry = Objects.requireNonNull(telemetry);
        this.styled = styled;
        telemetry.setDisplayFormat(styled ? Telemetry.DisplayFormat.HTML : Telemetry.DisplayFormat.CLASSIC);
        // Iterative OpModes also update automatically; retain the latest complete frame.
        telemetry.setAutoClear(false);
    }

    @Override
    public void beginFrame() {
        telemetry.clearAll();
    }

    @Override
    public void value(String label, Object value) {
        telemetry.addLine(styled
                ? "<b>" + escape(label) + ":</b> " + escape(String.valueOf(value))
                : label + ": " + value);
    }

    @Override
    public void coloredValue(String label, Object value, Color color) {
        Objects.requireNonNull(color);
        if (!styled) {
            value(label, value);
            return;
        }
        String hex = color == Color.RED ? "#FF6666" : "#66AAFF";
        telemetry.addLine("<b><font color='" + hex + "'>" + escape(label) + ": "
                + escape(String.valueOf(value)) + "</font></b>");
    }

    @Override
    public void number(String label, double value, String unit) {
        value(label, String.format(Locale.US, "%.2f%s", value, unit.isEmpty() ? "" : " " + unit));
    }

    @Override
    public void status(String label, String message, Level level) {
        Objects.requireNonNull(level);
        if (!styled) {
            value(label, "[" + level + "] " + message);
            return;
        }
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
                color = "#66CCFF";
        }
        telemetry.addLine("<b>" + escape(label) + ": <font color='" + color + "'>"
                + escape(message) + "</font></b>");
    }

    @Override
    public void line(String message) {
        telemetry.addLine(styled ? escape(message) : message);
    }

    @Override
    public void endFrame() {
        telemetry.update();
    }

    private static String escape(String text) {
        return text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
                .replace("\"", "&quot;").replace("'", "&#39;");
    }
}
