package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.BrakingTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.OTOSTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;
import org.firstinspires.ftc.teamcode.subsystems.RobotProfile;

public final class Tuning {
    private Tuning() { }

    @Tuner public static Procedure mecanum() { return new MecanumTuner(); }
    @Tuner public static Procedure otos() { return new OTOSTuner(); }
    @Tuner public static Procedure foresight() {
        return new ForesightTuner(PedroConstants::localizer, PedroConstants::drivetrain);
    }
    @Tuner public static Procedure braking() {
        return new BrakingTuner(PedroConstants::localizer, PedroConstants::drivetrain,
                () -> PedroConstants.foresightConfig(RobotProfile.loadActive(null)));
    }
    @Tuner public static Procedure tests() {
        return new Tests(PedroConstants::drivetrain, PedroConstants::localizer,
                () -> new Foresight(PedroConstants.foresightConfig(RobotProfile.loadActive(null))));
    }
}
