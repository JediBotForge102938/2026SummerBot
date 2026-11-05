package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherFeeder;

import java.util.Objects;

public class MechanismCoordinator {
    private static final double SHOOTING_INTAKE_ASSIST = 0.5;

    private final IntakeSubsystem intake;
    private final LauncherSubsystem launcher;
    private final LauncherFeeder feeder;

    public MechanismCoordinator(IntakeSubsystem intake, LauncherSubsystem launcher,
            LauncherFeeder feeder) {
        this.intake = intake;
        this.launcher = launcher;
        this.feeder = Objects.requireNonNull(feeder);
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
            intakePower += SHOOTING_INTAKE_ASSIST;
        }
        intake.setPower(intakePower);
    }

    public void stop() {
        launcher.update(false);
        feeder.update(false);
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
