package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherFeeder;

import java.util.Objects;

public class MechanismCoordinator {
    private final IntakeSubsystem intake;
    private final LauncherSubsystem launcher;
    private final LauncherFeeder feeder;
    private final double shootingIntakeAssist;

    public MechanismCoordinator(IntakeSubsystem intake, LauncherSubsystem launcher,
            LauncherFeeder feeder) {
        this(intake, launcher, feeder, 50);
    }

    public MechanismCoordinator(IntakeSubsystem intake, LauncherSubsystem launcher,
            LauncherFeeder feeder, int feedAssistPercent) {
        if (feedAssistPercent < 0 || feedAssistPercent > 100) {
            throw new IllegalArgumentException("Feed assist must be 0-100 percent");
        }
        this.intake = intake;
        this.launcher = launcher;
        this.feeder = Objects.requireNonNull(feeder);
        shootingIntakeAssist = feedAssistPercent / 100.0;
    }

    public void update(double intakePower, boolean shootRequested) {
        boolean ready;
        try {
            ready = launcher.update(shootRequested);
        } catch (IllegalStateException exception) {
            feeder.update(false);
            intake.setPower(0);
            throw exception;
        }
        feeder.update(ready);
        if (ready) {
            intakePower += shootingIntakeAssist;
        }
        intake.setPower(intakePower);
    }

    public void stop() {
        feeder.update(false);
        launcher.update(false);
        intake.setPower(0);
    }

    public void setLauncherTargetVelocity(int targetVelocity) {
        launcher.setTargetVelocity(targetVelocity);
        if (!launcher.isReady()) {
            feeder.update(false);
        }
    }

    public int getLauncherTargetVelocity() {
        return launcher.getTargetVelocity();
    }

    public boolean isLauncherReady() {
        return launcher.isReady();
    }
}
