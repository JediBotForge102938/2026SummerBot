package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.DrivePowers;
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
        // The simulator's Pedro sensor adapter already supplies robot-aligned axes.
        double mountingRotation = profile.isSimulatorRuntime() ? 0 : profile.otosMountingRotationDegrees;
        return otosConfig(profile.otosName, mountingRotation, profile.otosLinearScalar,
                profile.otosAngularScalar);
    }

    public static OTOSConfig otosConfig(String name, double mountingRotationDegrees,
            double linearScalar, double angularScalar) {
        return new OTOSConfig(c -> {
            c.name.set(name);
            c.offset.set(new Pose(0, 0, Math.toRadians(mountingRotationDegrees)));
            c.linearScalar.set(linearScalar);
            c.angularScalar.set(angularScalar);
        });
    }

    public static Mecanum drivetrain(HardwareMap hardware) {
        RobotProfile profile = RobotProfile.loadActive(null);
        MecanumConfig config = drivetrainConfig(profile);
        if ("starterbot".equals(profile.robot) && !profile.isSimulatorRuntime()) {
            return new Mecanum(hardware, config) {
                @Override
                public double[] computeWheelPowersUnnormalized(DrivePowers powers) {
                    // Physical StarterBot strafes opposite Pedro's standard wheel mix.
                    return super.computeWheelPowersUnnormalized(
                            new DrivePowers(powers.forward(), -powers.strafe(), powers.turn()));
                }
            };
        }
        return new Mecanum(hardware, config);
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
        return new ForesightConfig(c -> {
            Controller primaryForward = Controller.proportional(0.23768630058054682);
            Controller secondaryForward = Controller.proportional(0.08781873696959074);
            Controller primaryLateral = Controller.proportional(0.31737781957985134);
            Controller secondaryLateral = Controller.proportional(0.11726262384322846);
            c.forwardTranslational.set(Controller.piecewise(secondaryForward).put(2.5, primaryForward));
            c.strafeTranslational.set(Controller.piecewise(secondaryLateral).put(2.5, primaryLateral));
            c.coast.set(Controller.proportionalFeedforward(0.016644668065344603));
            c.brake.set(Controller.proportionalFeedforward(0.014147967855542912));
            c.headingFeedback.set(Controller.proportional(3.406971780703044));
            c.headingBrakeCoefficients.set(Vector2D.cartesian(0.12152906435570647, -0.0019175101025552008));
            c.linearBrakeCoefficients.set(Matrix.diag(0.06488131566156832, 0.058441664147441));
            c.quadraticBrakeCoefficients.set(Matrix.diag(0.0028877546739716022, 0.002741719582119601));
            c.maxAchievableForwardVelocity.set(63.80635933504575);
            c.maxAchievableStrafeVelocity.set(57.000575331950834);
            c.naturalForwardDeceleration.set(22.623603469226264);
            c.naturalStrafeDeceleration.set(31.7924778599749);
        });
    }

    public static ForesightConfig competitionForesightConfig() {
        return new ForesightConfig(c -> {
            Controller primaryForward = Controller.proportional(0.2661559228043619);
            Controller secondaryForward = Controller.proportional(0.09833750165897417);
            Controller primaryLateral = Controller.proportional(0.3739672162546907);
            Controller secondaryLateral = Controller.proportional(0.13817089381805375);
            c.forwardTranslational.set(Controller.piecewise(secondaryForward).put(2.5, primaryForward));
            c.strafeTranslational.set(Controller.piecewise(secondaryLateral).put(2.5, primaryLateral));
            c.coast.set(Controller.proportionalFeedforward(0.01800070834645365));
            c.brake.set(Controller.proportionalFeedforward(0.015300602094485603));
            c.headingFeedback.set(Controller.proportional(4.834660961741353));
            c.headingBrakeCoefficients.set(Vector2D.cartesian(0.8600445150401718, -0.3043591868677641));
            // Forward braking rerun: trial fit pending repeatability and motion checks.
            c.linearBrakeCoefficients.set(Matrix.diag(0.0429160763321019, 0.047777694467240193));
            c.quadraticBrakeCoefficients.set(Matrix.diag(0.0030923137162084838, 0.0017580708733732505));
            c.maxAchievableForwardVelocity.set(63.093824640738056);
            c.maxAchievableStrafeVelocity.set(50.71309816344906);
            c.naturalForwardDeceleration.set(42.42910859204445);
            c.naturalStrafeDeceleration.set(82.365058243127);
        });
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
