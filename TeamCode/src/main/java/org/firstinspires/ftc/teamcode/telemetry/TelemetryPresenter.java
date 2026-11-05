package org.firstinspires.ftc.teamcode.telemetry;

import java.util.Objects;

/** Composes arbitrary robot telemetry sources into one complete output frame. */
public class TelemetryPresenter {
    private final RobotTelemetry output;
    private final TelemetrySource[] sources;

    public TelemetryPresenter(RobotTelemetry output, TelemetrySource... sources) {
        this.output = Objects.requireNonNull(output);
        this.sources = sources.clone();
        for (TelemetrySource source : this.sources) {
            Objects.requireNonNull(source);
        }
    }

    public void publish() {
        output.beginFrame();
        for (TelemetrySource source : sources) {
            source.publishTelemetry(output);
        }
        output.endFrame();
    }
}
