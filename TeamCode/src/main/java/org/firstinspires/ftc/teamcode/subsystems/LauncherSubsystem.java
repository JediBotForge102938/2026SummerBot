package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.telemetry.RobotTelemetry;
import org.firstinspires.ftc.teamcode.telemetry.TelemetrySource;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Objects;
import java.util.Set;
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

    public static final class MotorConfig {
        public final String name;
        public final DcMotor.Direction direction;

        public MotorConfig(String name, DcMotor.Direction direction) {
            if (name == null || name.trim().isEmpty()) {
                throw new IllegalArgumentException("Launcher motor name must not be blank");
            }
            this.name = name;
            this.direction = Objects.requireNonNull(direction, "Launcher motor direction");
        }
    }

    private final List<MotorConfig> motorConfigs;
    private final List<DcMotorEx> motors = new ArrayList<>();
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
        this(Collections.singletonList(new MotorConfig("launcher", DcMotor.Direction.FORWARD)),
                nanoTime);
    }

    public LauncherSubsystem(List<MotorConfig> motorConfigs) {
        this(motorConfigs, System::nanoTime);
    }

    public LauncherSubsystem(List<MotorConfig> motorConfigs, LongSupplier nanoTime) {
        if (motorConfigs == null || motorConfigs.isEmpty()) {
            throw new IllegalArgumentException("At least one launcher motor is required");
        }
        Set<String> names = new HashSet<>();
        for (MotorConfig config : motorConfigs) {
            Objects.requireNonNull(config, "Launcher motor configuration");
            if (!names.add(config.name)) {
                throw new IllegalArgumentException("Duplicate launcher motor: " + config.name);
            }
        }
        this.motorConfigs = Collections.unmodifiableList(new ArrayList<>(motorConfigs));
        this.nanoTime = Objects.requireNonNull(nanoTime);
    }

    public void init(HardwareMap hardwareMap) {
        if (!motors.isEmpty()) {
            throw new IllegalStateException("LauncherSubsystem has already been initialized");
        }
        List<DcMotorEx> configured = new ArrayList<>();
        for (MotorConfig config : motorConfigs) {
            configured.add(Objects.requireNonNull(hardwareMap.get(DcMotorEx.class, config.name),
                    "Missing launcher motor: " + config.name));
        }
        for (int index = 0; index < configured.size(); index++) {
            DcMotorEx motor = configured.get(index);
            motor.setDirection(motorConfigs.get(index).direction);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(40, 0, 0, 12.5));
            motor.setVelocity(0);
        }
        motors.addAll(configured);
    }

    /**
     * Commands every motor in encoder ticks/second and reports combined feed readiness.
     */
    public boolean update(boolean launchRequested) {
        boolean enabled = launchRequested && targetVelocity > 0;
        requireInitialized();
        for (DcMotorEx motor : motors) {
            motor.setVelocity(enabled ? targetVelocity : 0);
        }

        if (!enabled) {
            resetReadiness();
        } else {
            double velocity;
            try {
                velocity = getVelocity();
            } catch (IllegalStateException exception) {
                resetReadiness();
                for (DcMotorEx motor : motors) {
                    motor.setVelocity(0);
                }
                throw exception;
            }
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

    private void requireInitialized() {
        if (motors.isEmpty()) {
            throw new IllegalStateException("LauncherSubsystem must be initialized before use");
        }
    }

    /** The slowest configured motor determines combined readiness. */
    public double getVelocity() {
        requireInitialized();
        double minimum = Double.POSITIVE_INFINITY;
        for (int index = 0; index < motors.size(); index++) {
            double velocity = motors.get(index).getVelocity();
            if (!Double.isFinite(velocity)) {
                throw new IllegalStateException("Invalid launcher velocity: "
                        + motorConfigs.get(index).name);
            }
            minimum = Math.min(minimum, velocity);
        }
        return minimum;
    }

    @Override
    public void publishTelemetry(RobotTelemetry output) {
        RobotTelemetry.Level level = status == Status.READY ? RobotTelemetry.Level.SUCCESS
                : status == Status.OFF ? RobotTelemetry.Level.INFO : RobotTelemetry.Level.WARNING;
        output.status("Launcher", status.name().replace('_', ' '), level);
        output.number("Launcher target", targetVelocity, "ticks/s");
        output.number("Launcher actual", getVelocity(), "ticks/s");
        for (int index = 0; index < motors.size(); index++) {
            output.number("Launcher " + motorConfigs.get(index).name,
                    motors.get(index).getVelocity(), "ticks/s");
        }
        if (targetVelocity == 0) {
            output.value("Launcher feed", "Disabled");
        } else {
            output.value("Launcher feed", "Above " + getFeedVelocityThreshold() + " ticks/s for "
                    + READY_DWELL_NANOS / 1_000_000 + " ms; stop at "
                    + getFeedDropoutThreshold() + " ticks/s");
        }
    }
}
