package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.BioBuzzLimelightClusterAdapter.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ConfigurationSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.telemetry.RobotTelemetry;
import org.firstinspires.ftc.teamcode.telemetry.TelemetrySource;

public class TeleopController implements TelemetrySource {
    private static final double PRECISION_POWER_SCALE = 0.35;
    private final DriveSubsystem drive;
    private final MechanismCoordinator mechanisms;
    private final DriveModePreferences preferences;
    private final HiveTrackingController tracking;
    private boolean fieldCentric;
    private boolean configurationManaged;
    private boolean chordArmed;
    private boolean chordConsumed;
    private boolean optionsWasDown;
    private boolean resetPending;
    private boolean dpadUpWasDown;
    private boolean dpadDownWasDown;
    private boolean launcherWasReady;
    private boolean missingTargetRumble;
    private boolean precisionDriving;
    private boolean aiming;
    private Alliance controllerLedAlliance;
    private double leftTrigger;
    private double rightTrigger;

    public TeleopController(DriveSubsystem drive, MechanismCoordinator mechanisms,
            DriveModePreferences preferences, HiveTrackingController tracking) {
        this.drive = drive;
        this.mechanisms = mechanisms;
        this.preferences = preferences;
        this.tracking = tracking;
        fieldCentric = preferences.isFieldCentric();
        mechanisms.setLauncherTargetVelocity(preferences.getLauncherTargetVelocity());
    }

    public void update(Gamepad gamepad) {
        if (!configurationManaged) {
            updateInit(gamepad);
        }
        precisionDriving = gamepad.left_bumper;
        aiming = gamepad.a;
        leftTrigger = gamepad.left_trigger;
        rightTrigger = gamepad.right_trigger;
        updateDriveMode(gamepad.back || gamepad.share, gamepad.start || gamepad.options);
        updateLauncherTarget(gamepad.dpad_up, gamepad.dpad_down);

        double forward = -gamepad.left_stick_y;
        double strafe = gamepad.left_stick_x;
        double trackingRotation = tracking.update(gamepad.a);
        double rotate = gamepad.a ? trackingRotation : gamepad.right_stick_x;
        double powerScale = precisionDriving ? PRECISION_POWER_SCALE : 1;
        if (gamepad.a) {
            forward *= PRECISION_POWER_SCALE;
            strafe *= PRECISION_POWER_SCALE;
            // While aiming, precision applies only to translation, not automatic rotation.
            powerScale = 1;
        }
        if (fieldCentric) {
            if (gamepad.a) {
                drive.driveFieldRelative(forward, strafe, rotate, powerScale, true);
            } else {
                drive.driveFieldRelative(forward, strafe, rotate, powerScale);
            }
        } else {
            if (gamepad.a) {
                drive.driveRobotRelative(forward, strafe, rotate, powerScale, true);
            } else {
                drive.driveRobotRelative(forward, strafe, rotate, powerScale);
            }
        }
        mechanisms.update(gamepad.right_trigger - gamepad.left_trigger, gamepad.right_bumper);
        boolean ready = mechanisms.isLauncherReady();
        boolean missingTarget = aiming && !tracking.hasTarget();
        if (missingTarget) {
            if (!missingTargetRumble) {
                gamepad.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
            }
        } else if (missingTargetRumble) {
            gamepad.stopRumble();
        } else if (ready && !launcherWasReady) {
            gamepad.rumble(200);
        } else if (!ready && launcherWasReady) {
            gamepad.stopRumble();
        }
        missingTargetRumble = missingTarget;
        launcherWasReady = ready;
    }

    public boolean isFieldCentric() {
        return fieldCentric;
    }

    public void configure(ConfigurationSubsystem configuration, Gamepad gamepad) {
        if (!configuration.isLocked()) {
            throw new IllegalStateException("Lock configuration before applying it");
        }
        fieldCentric = configuration.isFieldCentric();
        tracking.setAlliance(configuration.getAlliance());
        configurationManaged = true;
        dpadUpWasDown = gamepad.dpad_up;
        dpadDownWasDown = gamepad.dpad_down;
        chordArmed = false;
        chordConsumed = false;
        optionsWasDown = false;
        resetPending = false;
    }

    /** Allows alliance selection during INIT without driving or spinning mechanisms. */
    public void updateInit(Gamepad gamepad) {
        if (configurationManaged) {
            return;
        }
        tracking.selectAlliance(gamepad.dpad_left, gamepad.dpad_right);
        Alliance selectedAlliance = tracking.getAlliance();
        if (selectedAlliance != controllerLedAlliance) {
            gamepad.setLedColor(selectedAlliance == Alliance.RED ? 1 : 0, 0,
                    selectedAlliance == Alliance.BLUE ? 1 : 0, Gamepad.LED_DURATION_CONTINUOUS);
            controllerLedAlliance = selectedAlliance;
        }
    }

    public void stop() {
        drive.driveRobotRelative(0, 0, 0);
        mechanisms.stop();
        tracking.stop();
        launcherWasReady = false;
        missingTargetRumble = false;
        precisionDriving = false;
        aiming = false;
        leftTrigger = 0;
        rightTrigger = 0;
    }

    @Override
    public void publishTelemetry(RobotTelemetry output) {
        output.status("Drive mode", fieldCentric ? "FIELD-CENTRIC" : "ROBOT-CENTRIC",
                RobotTelemetry.Level.INFO);
        output.status("Driving", aiming ? "AIMING (SLOW TRANSLATION)"
                        : precisionDriving ? "PRECISION" : "NORMAL",
                aiming || precisionDriving ? RobotTelemetry.Level.WARNING : RobotTelemetry.Level.INFO);
        output.number("Drive power limit", (!aiming && precisionDriving
                ? PRECISION_POWER_SCALE : 1) * 100, "%");
        output.number("Translation input scale", (aiming || precisionDriving
                ? PRECISION_POWER_SCALE : 1) * 100, "%");
        output.number("Left trigger", leftTrigger, "");
        output.number("Right trigger", rightTrigger, "");
        output.line("Left bumper: hold for precision driving");
        output.line("D-pad up/down: launcher target +/-" + LauncherSubsystem.TARGET_VELOCITY_STEP
                + " ticks/s (0-" + LauncherSubsystem.MAX_TARGET_VELOCITY + "); saved automatically");
        output.line("Share+Options / Back+Start: toggle drive mode (saved automatically)");
        output.line("Options / Start alone: release to reset heading in field-centric mode");
    }

    private void updateLauncherTarget(boolean upDown, boolean downDown) {
        int direction = 0;
        if (upDown && !downDown && !dpadUpWasDown) {
            direction = 1;
        } else if (downDown && !upDown && !dpadDownWasDown) {
            direction = -1;
        }
        dpadUpWasDown = upDown;
        dpadDownWasDown = downDown;

        if (direction != 0) {
            int current = mechanisms.getLauncherTargetVelocity();
            int target = Math.max(0, Math.min(LauncherSubsystem.MAX_TARGET_VELOCITY,
                    current + direction * LauncherSubsystem.TARGET_VELOCITY_STEP));
            if (target != current) {
                preferences.saveLauncherTargetVelocity(target);
                mechanisms.setLauncherTargetVelocity(target);
            }
        }
    }

    private void updateDriveMode(boolean shareDown, boolean optionsDown) {
        if (chordArmed && shareDown && optionsDown) {
            preferences.saveFieldCentric(!fieldCentric);
            fieldCentric = !fieldCentric;
            chordArmed = false;
            chordConsumed = true;
            resetPending = false;
        }

        if (optionsDown && !optionsWasDown) {
            resetPending = chordArmed && !shareDown && !chordConsumed;
        }
        // Waiting for release distinguishes a standalone press from an Options-first chord.
        if (!optionsDown && optionsWasDown) {
            if (resetPending && !chordConsumed && fieldCentric) {
                drive.resetHeading();
            }
            resetPending = false;
        }
        optionsWasDown = optionsDown;

        // Also suppress buttons held across INIT/START until both have been released.
        if (!shareDown && !optionsDown) {
            chordArmed = true;
            chordConsumed = false;
        }
    }
}
