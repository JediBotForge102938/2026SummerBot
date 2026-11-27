package org.firstinspires.ftc.teamcode.pedro.procedures;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.Telemetry;

final class BrakingDiagnostics {
    private final Telemetry telemetry;
    private final String axis;
    private double lastSample = Double.NEGATIVE_INFINITY;

    BrakingDiagnostics(Telemetry telemetry, String axis) {
        this.telemetry = telemetry;
        this.axis = axis;
    }

    void record(double seconds, String state, int trial, Localizer localizer, DrivePowers command) {
        if (seconds - lastSample < 0.1) return;
        lastSample = seconds;
        Pose pose = localizer.pose();
        Velocity velocity = localizer.velocity();
        telemetry.addData("Braking test", "%s trial %d %s", axis, trial + 1, state);
        telemetry.addData("Position (in)", "X %.3f Y %.3f", pose.x(), pose.y());
        telemetry.addData("Velocity (in/s)", "X %.3f Y %.3f", velocity.vx, velocity.vy);
        telemetry.addData("Heading (deg)", "%.3f", Math.toDegrees(pose.heading()));
        telemetry.addData("Angular velocity (deg/s)", "%.3f", Math.toDegrees(velocity.omega));
        telemetry.addData("Commanded power", "Forward %.4f Strafe %.4f Turn %.4f",
                command.forward(), command.strafe(), command.turn());
        telemetry.update();
        RobotLog.ii("PedroBraking",
                "axis=%s t=%.3f trial=%d state=%s x=%.3f y=%.3f vx=%.3f vy=%.3f headingRad=%.5f omegaRad=%.5f forward=%.5f strafe=%.5f turn=%.5f",
                axis, seconds, trial + 1, state, pose.x(), pose.y(), velocity.vx, velocity.vy,
                pose.heading(), velocity.omega, command.forward(), command.strafe(), command.turn());
    }
}
