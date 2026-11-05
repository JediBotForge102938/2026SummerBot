package org.firstinspires.ftc.teamcode.telemetry;

/**
 * Presentation-independent output. All labels, values, units, and messages are plain text.
 * The caller owns a complete frame: beginFrame, publish all sources, then endFrame.
 * Implementations own formatting and transport; publishers must not flush or send HTML.
 */
public interface RobotTelemetry {
    enum Level {
        INFO, SUCCESS, WARNING, ERROR
    }

    enum Color {
        RED, BLUE
    }

    void beginFrame();

    void value(String label, Object value);

    void coloredValue(String label, Object value, Color color);

    void number(String label, double value, String unit);

    void status(String label, String message, Level level);

    void line(String message);

    void endFrame();
}
