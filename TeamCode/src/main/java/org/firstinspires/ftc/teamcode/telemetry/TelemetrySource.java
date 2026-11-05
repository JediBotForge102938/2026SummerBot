package org.firstinspires.ftc.teamcode.telemetry;

/** Publishes current state without changing hardware, control state, or frame lifecycle. */
public interface TelemetrySource {
    void publishTelemetry(RobotTelemetry output);
}
