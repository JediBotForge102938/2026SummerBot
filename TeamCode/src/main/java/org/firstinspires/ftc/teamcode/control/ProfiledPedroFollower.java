package org.firstinspires.ftc.teamcode.control;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.paths.PathConstraints;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotProfile;
import org.firstinspires.ftc.teamcode.subsystems.TrackingSubsystem;

/** Shared hardware/output configuration for navigation and chassis tuning. */
public final class ProfiledPedroFollower {
    private ProfiledPedroFollower() { }

    public static Follower create(DriveSubsystem drive, TrackingSubsystem tracking,
            RobotProfile profile, double headingResponseScale) {
        if (drive == null || tracking == null || profile == null || profile.navigation == null) {
            throw new IllegalArgumentException("Pedro requires drive, tracking and navigation settings");
        }
        if (!Double.isFinite(headingResponseScale) || headingResponseScale <= 0) {
            throw new IllegalArgumentException("Heading response scale must be finite and positive");
        }
        RobotProfile.NavigationSettings settings = profile.navigation;
        FollowerConstants constants = new FollowerConstants();
        constants.defaults();
        constants.setCoefficientsTranslationalPIDF(
                new PIDFCoefficients(settings.translationalP, 0,
                        settings.translationalD, settings.translationalF));
        constants.setCoefficientsHeadingPIDF(
                new PIDFCoefficients(settings.headingP * headingResponseScale,
                        0, settings.headingD, settings.headingF));
        constants.setCoefficientsDrivePIDF(
                new FilteredPIDFCoefficients(settings.driveP, 0,
                        settings.driveD, 0, settings.driveF));
        constants.setMass(settings.massKg);
        constants.setForwardZeroPowerAcceleration(settings.forwardZeroPowerAcceleration);
        constants.setLateralZeroPowerAcceleration(settings.lateralZeroPowerAcceleration);
        constants.setAutomaticHoldEnd(true);
        PedroDriveOutput output = new PedroDriveOutput(drive);
        output.setXVelocity(settings.forwardVelocity);
        output.setYVelocity(settings.lateralVelocity);
        PathConstraints constraints = PathConstraints.defaultConstraints.copy();
        constraints.setTranslationalConstraint(settings.arrivalToleranceIn);
        constraints.setHeadingConstraint(Math.toRadians(settings.headingToleranceDegrees));
        constraints.setVelocityConstraint(settings.stationarySpeedInPerSec);
        Follower follower = new Follower(constants, new PedroTrackingLocalizer(tracking),
                output, constraints);
        follower.setMaxPowerScaling(settings.transitPower);
        return follower;
    }
}
