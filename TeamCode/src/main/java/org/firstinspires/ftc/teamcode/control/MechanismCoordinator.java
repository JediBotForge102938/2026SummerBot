package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;

public class MechanismCoordinator {
    private static final double SHOOTING_INTAKE_ASSIST = 0.5;

    private final IntakeSubsystem intake;
    private final LauncherSubsystem launcher;

    public MechanismCoordinator(IntakeSubsystem intake, LauncherSubsystem launcher) {
        this.intake = intake;
        this.launcher = launcher;
    }

    public void update(double intakePower, boolean shootRequested) {
        if (launcher.update(shootRequested)) {
            intakePower += SHOOTING_INTAKE_ASSIST;
        }
        intake.setPower(intakePower);
    }

    public void stop() {
        launcher.update(false);
        intake.setPower(0);
    }

    public void setLauncherTargetVelocity(int targetVelocity) {
        launcher.setTargetVelocity(targetVelocity);
    }

    public int getLauncherTargetVelocity() {
        return launcher.getTargetVelocity();
    }

    public boolean isLauncherReady() {
        return launcher.isReady();
    }
}
