package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotProfile;
import org.firstinspires.ftc.teamcode.telemetry.TelemetryTable;

import java.util.Locale;

@TeleOp(name = "Launcher Tuning", group = "2 - (Tests)")
public class LauncherTuningTeleOp extends OpMode {
    private enum Row { TARGET, P, I, D, F }

    private RobotProfile profile;
    private LauncherSubsystem launcher;
    private GateSubsystem gate;
    private boolean launcherInitialized;
    private boolean gateInitialized;
    private Row row = Row.TARGET;
    private boolean up, down, left, right;
    private boolean spinArmed;
    private boolean spinning;

    @Override
    public void init() {
        profile = RobotProfile.loadActive(null);
        if (profile.launcherGate != null) {
            gate = new GateSubsystem(profile.launcherGate);
            gate.init(hardwareMap);
            gateInitialized = true;
        }
        launcher = new LauncherSubsystem(profile.launcherMotors, profile.launcherPidf,
                profile.launcherFeedVelocityMargin);
        launcher.init(hardwareMap);
        launcherInitialized = true;
        launcher.setTargetVelocity(1200);
        telemetry.setDisplayFormat(org.firstinspires.ftc.robotcore.external.Telemetry.DisplayFormat.HTML);
        rememberArrows();
        publish();
    }

    @Override
    public void init_loop() {
        rememberArrows();
        publish();
    }

    @Override
    public void start() {
        rememberArrows();
        spinArmed = !gamepad1.right_bumper;
        publish();
    }

    @Override
    public void loop() {
        boolean moveUp = gamepad1.dpad_up && !up && !gamepad1.dpad_down;
        boolean moveDown = gamepad1.dpad_down && !down && !gamepad1.dpad_up;
        boolean decrease = gamepad1.dpad_left && !left && !gamepad1.dpad_right;
        boolean increase = gamepad1.dpad_right && !right && !gamepad1.dpad_left;
        Row[] rows = Row.values();
        if (moveUp != moveDown) {
            row = rows[(row.ordinal() + (moveUp ? rows.length - 1 : 1)) % rows.length];
        }
        if (increase != decrease) {
            adjust(increase ? 1 : -1);
        }
        rememberArrows();
        if (!gamepad1.right_bumper) {
            spinArmed = true;
        }
        spinning = spinArmed && gamepad1.right_bumper && launcher.getTargetVelocity() > 0;
        launcher.update(spinning);
        publish();
    }

    private double coefficientStep() {
        double step = row == Row.I ? 0.01 : row == Row.P ? 1.0 : 0.1;
        return gamepad1.left_bumper ? step / 10 : step;
    }

    private void adjust(int direction) {
        if (row == Row.TARGET) {
            launcher.setTargetVelocity(Math.max(0, Math.min(LauncherSubsystem.MAX_TARGET_VELOCITY,
                    launcher.getTargetVelocity() + direction * LauncherSubsystem.TARGET_VELOCITY_STEP)));
            return;
        }
        LauncherSubsystem.PidfConfig current = launcher.getPidf();
        double[] values = {current.p, current.i, current.d, current.f};
        int index = row.ordinal() - 1;
        values[index] = Math.max(0, values[index] + direction * coefficientStep());
        launcher.setPidf(new LauncherSubsystem.PidfConfig(values[0], values[1], values[2], values[3]));
    }

    private void rememberArrows() {
        up = gamepad1.dpad_up;
        down = gamepad1.dpad_down;
        left = gamepad1.dpad_left;
        right = gamepad1.dpad_right;
    }

    private void publish() {
        telemetry.clearAll();
        telemetry.addData("Robot profile", profile.name);
        TelemetryTable.heading(telemetry, "Launcher tuning");
        TelemetryTable.row(telemetry, "Target", launcher.getTargetVelocity() + " ticks/s",
                row == Row.TARGET, null);
        LauncherSubsystem.PidfConfig current = launcher.getPidf();
        double[] values = {current.p, current.i, current.d, current.f};
        for (int index = 0; index < values.length; index++) {
            Row parameter = Row.values()[index + 1];
            TelemetryTable.row(telemetry, parameter.name(),
                    String.format(Locale.US, "%.4f", values[index]), row == parameter, null);
        }
        double actual = launcher.getVelocity();
        telemetry.addData("Actual / target", "%.1f / %d ticks/s", actual,
                launcher.getTargetVelocity());
        telemetry.addData("Error (target - actual)", "%.1f ticks/s",
                launcher.getTargetVelocity() - actual);
        telemetry.addData("Tracking (+/-50 ticks/s)", !spinning ? "OFF"
                : Math.abs(launcher.getTargetVelocity() - actual) <= 50 ? "WITHIN" : "OUTSIDE");
        telemetry.addData("Spin command", spinning ? "ON" : "OFF");
        telemetry.addData("Readiness", launcher.getStatus());
        telemetry.addData("Feed threshold", "> %d ticks/s for 200 ms (profile margin %d)",
                launcher.getFeedVelocityThreshold(), profile.launcherFeedVelocityMargin);
        telemetry.addData("Adjustment step", row == Row.TARGET ? "25 ticks/s"
                : String.format(Locale.US, "%.4f", coefficientStep()));
        telemetry.addLine("After START: D-pad up/down selects; left/right adjusts (one step per press).");
        telemetry.addLine("Hold RB to spin; release to stop. Hold LB for fine PIDF steps.");
        telemetry.addLine("No feeding. Changes are temporary; copy PIDF to the profile to keep them.");
        telemetry.update();
    }

    @Override
    public void stop() {
        if (launcherInitialized) {
            launcher.update(false);
        }
        if (gateInitialized) {
            gate.update(false);
        }
    }
}
