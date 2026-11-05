package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.telemetry.RobotTelemetry;
import org.firstinspires.ftc.teamcode.telemetry.TelemetrySource;

import java.util.function.LongSupplier;

public class LauncherSubsystem implements TelemetrySource {
    public static final int DEFAULT_TARGET_VELOCITY = 1250;
    public static final int MAX_TARGET_VELOCITY = 6000;
    public static final int TARGET_VELOCITY_STEP = 25;
    private static final int FEED_VELOCITY_MARGIN = 50;
    private static final int FEED_HYSTERESIS = 25;
    private static final long READY_DWELL_NANOS = 200_000_000L;

    public enum Status {
        OFF, SPINNING_UP, READY, BELOW_TARGET
    }

    private DcMotorEx launcher;
    private CRServo windmillServo;
    private int targetVelocity = DEFAULT_TARGET_VELOCITY;
    private final LongSupplier nanoTime;
    private boolean timingReadiness;
    private long readySince;
    private boolean ready;
    private boolean hasBeenReady;
    private Status status = Status.OFF;

    public LauncherSubsystem() {
        this(System::nanoTime);
    }

    public LauncherSubsystem(LongSupplier nanoTime) {
        this.nanoTime = nanoTime;
    }

    public void init(HardwareMap hardwareMap) {
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmillServo = hardwareMap.get(CRServo.class, "windmillServo");

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(40, 0, 0, 12.5));

        windmillServo.setPower(0);
        windmillServo.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    /**
     * Updates launcher and windmill output and reports whether the windmill is running.
     */
    public boolean update(boolean launchRequested) {
        boolean enabled = launchRequested && targetVelocity > 0;
        if (enabled) {
            launcher.setVelocity(targetVelocity);
        } else {
            launcher.setVelocity(0);
        }

        if (!enabled) {
            resetReadiness();
        } else {
            double velocity = launcher.getVelocity();
            if (ready && velocity <= getFeedDropoutThreshold()) {
                ready = false;
                timingReadiness = false;
            }
            if (!ready) {
                if (velocity > getFeedVelocityThreshold()) {
                    long now = nanoTime.getAsLong();
                    if (!timingReadiness) {
                        readySince = now;
                        timingReadiness = true;
                    }
                    ready = now - readySince >= READY_DWELL_NANOS;
                } else {
                    timingReadiness = false;
                }
            }
            hasBeenReady |= ready;
            status = ready ? Status.READY
                    : hasBeenReady ? Status.BELOW_TARGET : Status.SPINNING_UP;
        }
        windmillServo.setPower(ready ? 1 : 0);
        return ready;
    }

    public void setTargetVelocity(int targetVelocity) {
        if (targetVelocity < 0 || targetVelocity > MAX_TARGET_VELOCITY) {
            throw new IllegalArgumentException("Launcher target must be 0-"
                    + MAX_TARGET_VELOCITY + " encoder ticks/second");
        }
        if (this.targetVelocity != targetVelocity) {
            this.targetVelocity = targetVelocity;
            resetReadiness();
            if (windmillServo != null) {
                windmillServo.setPower(0);
            }
        }
    }

    public int getTargetVelocity() {
        return targetVelocity;
    }

    public int getFeedVelocityThreshold() {
        return Math.max(0, targetVelocity - FEED_VELOCITY_MARGIN);
    }

    public int getFeedDropoutThreshold() {
        return Math.max(0, getFeedVelocityThreshold() - FEED_HYSTERESIS);
    }

    public boolean isReady() {
        return ready;
    }

    public Status getStatus() {
        return status;
    }

    private void resetReadiness() {
        ready = false;
        timingReadiness = false;
        hasBeenReady = false;
        status = Status.OFF;
    }

    public double getVelocity() {
        return launcher.getVelocity();
    }

    @Override
    public void publishTelemetry(RobotTelemetry output) {
        RobotTelemetry.Level level = status == Status.READY ? RobotTelemetry.Level.SUCCESS
                : status == Status.OFF ? RobotTelemetry.Level.INFO : RobotTelemetry.Level.WARNING;
        output.status("Launcher", status.name().replace('_', ' '), level);
        output.number("Launcher target", targetVelocity, "ticks/s");
        output.number("Launcher actual", getVelocity(), "ticks/s");
        if (targetVelocity == 0) {
            output.value("Launcher feed", "Disabled");
        } else {
            output.value("Launcher feed", "Above " + getFeedVelocityThreshold() + " ticks/s for "
                    + READY_DWELL_NANOS / 1_000_000 + " ms; stop at "
                    + getFeedDropoutThreshold() + " ticks/s");
        }
    }
}
