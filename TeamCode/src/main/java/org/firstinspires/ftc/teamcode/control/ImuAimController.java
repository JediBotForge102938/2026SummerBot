package org.firstinspires.ftc.teamcode.control;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.function.DoubleSupplier;
import java.util.function.LongSupplier;

/** Integrates measured yaw rate over the estimated age of a camera observation. */
public final class ImuAimController {
    private static final long HISTORY_NS = 500_000_000L;
    private static final long MAX_SAMPLE_GAP_NS = 100_000_000L;
    private static final double RATE_GAIN = 0.002;
    private final DoubleSupplier yawRate;
    private final LongSupplier clock;
    private final Deque<Sample> samples = new ArrayDeque<>();
    private long now;
    private double rate;
    private double correctedAngle;
    private boolean compensated;

    public ImuAimController(DoubleSupplier yawRate) {
        this(yawRate, System::nanoTime);
    }

    ImuAimController(DoubleSupplier yawRate, LongSupplier clock) {
        this.yawRate = yawRate;
        this.clock = clock;
    }

    public void sample() {
        now = clock.getAsLong();
        rate = yawRate.getAsDouble();
        if (!Double.isFinite(rate)) {
            reset();
            throw new IllegalStateException("Tracking IMU yaw rate must be finite");
        }
        Sample last = samples.peekLast();
        if (last != null && (now <= last.time || now - last.time > MAX_SAMPLE_GAP_NS)) {
            samples.clear();
        }
        samples.addLast(new Sample(now, rate));
        while (samples.size() > 2 && now - samples.peekFirst().time > HISTORY_NS) {
            samples.removeFirst();
        }
    }

    public double turn(double cameraAngle, double observationAgeMs) {
        if (!Double.isFinite(cameraAngle) || !Double.isFinite(observationAgeMs)
                || observationAgeMs < 0) {
            throw new IllegalArgumentException("Tracking angle and observation age must be finite");
        }
        if (samples.isEmpty()) {
            throw new IllegalStateException("Sample the tracking IMU before aiming");
        }
        long ageNs = (long) (observationAgeMs * 1_000_000);
        compensated = ageNs <= HISTORY_NS && now - samples.peekFirst().time >= ageNs;
        double rotation = 0;
        if (compensated) {
            long captureTime = now - ageNs;
            Sample previous = null;
            for (Sample sample : samples) {
                if (previous != null && sample.time > captureTime) {
                    long start = Math.max(captureTime, previous.time);
                    double startRate = previous.rate + (sample.rate - previous.rate)
                            * ((double) (start - previous.time) / (sample.time - previous.time));
                    rotation += (startRate + sample.rate) * 0.5
                            * (sample.time - start) / 1_000_000_000.0;
                }
                previous = sample;
            }
        }
        // Camera-right bearing increases with CCW yaw, including for the rear-facing camera.
        correctedAngle = cameraAngle + rotation;
        double power = HiveTrackingController.getTurnPower(correctedAngle);
        if (Math.abs(correctedAngle) <= 2 && Math.abs(rate) <= 5) {
            return 0;
        }
        // Positive drive rotation is clockwise, opposite the IMU's positive yaw rate.
        return Math.max(-0.60, Math.min(0.60, power + RATE_GAIN * rate));
    }

    public boolean isCompensated() {
        return compensated;
    }

    public double getCorrectedAngle() {
        return correctedAngle;
    }

    public void reset() {
        samples.clear();
        compensated = false;
        correctedAngle = 0;
        rate = 0;
    }

    private static final class Sample {
        final long time;
        final double rate;

        Sample(long time, double rate) {
            this.time = time;
            this.rate = rate;
        }
    }
}
