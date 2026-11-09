package org.firstinspires.ftc.teamcode.control;

import com.pedropathing.drivetrain.CustomDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/** Pedro-only output adapter; all motor writes remain through the shared chassis mixer. */
final class PedroDriveOutput extends CustomDrivetrain {
    private final DriveSubsystem drive;
    private double xVelocity;
    private double yVelocity;

    PedroDriveOutput(DriveSubsystem drive) {
        this.drive = drive;
    }

    @Override public void arcadeDrive(double robotForward, double robotLeft,
            double counterClockwise) {
        if (!Double.isFinite(robotForward) || !Double.isFinite(robotLeft)
                || !Double.isFinite(counterClockwise)) {
            breakFollowing();
            throw new IllegalStateException("Pedro produced an invalid drive command");
        }
        if (maxPowerScaling == 0) {
            breakFollowing();
            return;
        }
        drive.driveRobotRelative(robotForward / maxPowerScaling, -robotLeft / maxPowerScaling,
                -counterClockwise / maxPowerScaling, maxPowerScaling, true);
    }

    @Override public void updateConstants() { }
    @Override public void breakFollowing() { drive.driveRobotRelative(0, 0, 0); }
    @Override public void runDrive(double[] powers) {
        breakFollowing();
        throw new UnsupportedOperationException("Pedro custom drive requires vector commands");
    }
    @Override public void startTeleopDrive() { }
    @Override public void startTeleopDrive(boolean brake) { }
    @Override public double xVelocity() { return xVelocity; }
    @Override public double yVelocity() { return yVelocity; }
    @Override public void setXVelocity(double velocity) { xVelocity = velocity; }
    @Override public void setYVelocity(double velocity) { yVelocity = velocity; }
    @Override public double getVoltage() { return 12.0; }
    @Override public String debugString() {
        return "Pedro full-power forward/lateral speeds: " + xVelocity + "/" + yVelocity + " in/s";
    }
}
