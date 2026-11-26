package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.OTOSConfig;
import com.pedropathing.revhub.localizers.OTOSLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.subsystems.RobotProfile;

public final class PedroConstants {
    private PedroConstants() { }

    public static MecanumConfig drivetrainConfig(RobotProfile profile) {
        if ("starterbot".equals(profile.robot)) {
            // The simulator swaps same-side wheels to model StarterBot's legacy mixer.
            return profile.isSimulatorRuntime()
                    ? initialDrivetrainConfig(true) : starterDrivetrainConfig();
        }
        if ("competitionbot".equals(profile.robot)) return competitionDrivetrainConfig();
        throw new IllegalArgumentException("Unsupported Pedro robot: " + profile.robot);
    }

    public static MecanumConfig starterDrivetrainConfig() {
        return initialDrivetrainConfig(false);
    }

    public static MecanumConfig competitionDrivetrainConfig() {
        return initialDrivetrainConfig(false);
    }

    private static MecanumConfig initialDrivetrainConfig(boolean simulatorStarterWheelSwap) {
        return new MecanumConfig(c -> {
            c.frontLeftName.set(simulatorStarterWheelSwap ? "left_back_drive" : "left_front_drive");
            c.backLeftName.set(simulatorStarterWheelSwap ? "left_front_drive" : "left_back_drive");
            c.frontRightName.set(simulatorStarterWheelSwap ? "right_back_drive" : "right_front_drive");
            c.backRightName.set(simulatorStarterWheelSwap ? "right_front_drive" : "right_back_drive");
            c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
            c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
            c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
            c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        });
    }

    public static OTOSConfig otosConfig(RobotProfile profile) {
        if (profile.isSimulatorRuntime()) return initialOtosConfig(profile);
        if ("starterbot".equals(profile.robot)) return starterOtosConfig(profile);
        if ("competitionbot".equals(profile.robot)) return competitionOtosConfig(profile);
        throw new IllegalArgumentException("Unsupported Pedro robot: " + profile.robot);
    }

    public static OTOSConfig starterOtosConfig(RobotProfile profile) {
        return initialOtosConfig(profile);
    }

    public static OTOSConfig competitionOtosConfig(RobotProfile profile) {
        return initialOtosConfig(profile);
    }

    private static OTOSConfig initialOtosConfig(RobotProfile profile) {
        return new OTOSConfig(c -> {
            c.name.set(profile.otosName);
            c.offset.set(Pose.zero());
            c.linearScalar.set(profile.otosLinearScalar);
            c.angularScalar.set(profile.otosAngularScalar);
        });
    }

    public static Mecanum drivetrain(HardwareMap hardware) {
        return new Mecanum(hardware, drivetrainConfig(RobotProfile.loadActive(null)));
    }

    public static OTOSLocalizer localizer(HardwareMap hardware) {
        return new OTOSLocalizer(hardware, otosConfig(RobotProfile.loadActive(null)));
    }

    public static ForesightConfig foresightConfig(RobotProfile profile) {
        if (!profile.isSimulatorRuntime()) {
            if ("starterbot".equals(profile.robot)) return starterForesightConfig();
            if ("competitionbot".equals(profile.robot)) return competitionForesightConfig();
            throw new IllegalArgumentException("Unsupported Pedro robot: " + profile.robot);
        }
        return new ForesightConfig(c -> {
            // Simulator-specific values for its near-instant motor response.
            c.forwardTranslational.set(Controller.proportional(0.08));
            c.strafeTranslational.set(Controller.proportional(0.08));
            c.coast.set(Controller.proportionalFeedforward(0.0171));
            c.brake.set(Controller.proportionalFeedforward(0.005));
            c.headingFeedback.set(Controller.proportional(3.14));
            c.headingBrakeCoefficients.set(Vector2D.cartesian(0.045, 0.0022));
            c.linearBrakeCoefficients.set(Matrix.diag(0, 0));
            c.quadraticBrakeCoefficients.set(Matrix.diag(0, 0));
            c.maxAchievableForwardVelocity.set(58.44);
            c.maxAchievableStrafeVelocity.set(58.44);
            c.naturalForwardDeceleration.set(3000.0);
            c.naturalStrafeDeceleration.set(3000.0);
            c.maxVelocityConstraint.set(20.0);
        });
    }

    public static ForesightConfig starterForesightConfig() {
        throw new IllegalStateException("Run AutoTune and add StarterBot Foresight configuration");
    }

    public static ForesightConfig competitionForesightConfig() {
        throw new IllegalStateException("Run AutoTune and add CompetitionBot Foresight configuration");
    }

    public static Follower create(HardwareMap hardware) {
        RobotProfile profile = RobotProfile.loadActive(null);
        ForesightConfig config = foresightConfig(profile);
        // Waypoint routes must not intentionally skip their intermediate segments.
        config.pathSkip.set(false);
        Foresight algorithm = new Foresight(config);
        return new Follower(localizer(hardware), drivetrain(hardware), algorithm);
    }
}
