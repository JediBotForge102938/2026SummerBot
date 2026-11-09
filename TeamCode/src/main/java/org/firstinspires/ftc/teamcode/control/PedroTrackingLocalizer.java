package org.firstinspires.ftc.teamcode.control;

import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;
import org.firstinspires.ftc.teamcode.subsystems.FieldMap;
import org.firstinspires.ftc.teamcode.subsystems.TrackingSubsystem;

/** Bridges the established OTOS + hub-IMU estimate into Pedro's follower. */
final class PedroTrackingLocalizer implements Localizer {
    private final TrackingSubsystem tracking;

    PedroTrackingLocalizer(TrackingSubsystem tracking) {
        this.tracking = tracking;
    }

    @Override public Pose getPose() {
        FieldMap.Pose pose = tracking.getPose();
        if (pose == null) return new Pose(Double.NaN, Double.NaN, Double.NaN);
        return new Pose(pose.x, pose.y, Math.toRadians(pose.headingDegrees));
    }

    @Override public Pose getVelocity() {
        return new Pose(tracking.getVelocityX(), tracking.getVelocityY(),
                Math.toRadians(tracking.getHeadingRateDegreesPerSecond()));
    }

    @Override public Vector getVelocityVector() {
        Vector velocity = new Vector();
        velocity.setOrthogonalComponents(tracking.getVelocityX(), tracking.getVelocityY());
        return velocity;
    }

    @Override public void setStartPose(Pose pose) { }
    @Override public void setPose(Pose pose) { }
    @Override public void update() { }
    @Override public double getTotalHeading() { return getPose().getHeading(); }
    @Override public double getForwardMultiplier() { return 1; }
    @Override public double getLateralMultiplier() { return 1; }
    @Override public double getTurningMultiplier() { return 1; }
    @Override public void resetIMU() { }
    @Override public double getIMUHeading() { return getPose().getHeading(); }
    @Override public boolean isNAN() {
        Pose pose = getPose();
        Pose velocity = getVelocity();
        return !Double.isFinite(pose.getX()) || !Double.isFinite(pose.getY())
                || !Double.isFinite(pose.getHeading()) || !Double.isFinite(velocity.getX())
                || !Double.isFinite(velocity.getY()) || !Double.isFinite(velocity.getHeading());
    }
}
