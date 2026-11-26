package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.BioBuzzLimelightClusterAdapter.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ConfigurationSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.telemetry.RobotTelemetry;
import org.firstinspires.ftc.teamcode.telemetry.TelemetrySource;

public class TeleopController implements TelemetrySource {
    private double driveSpeed = 1;
    private double bumperSpeed = 0.35;
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
    private boolean dpadLeftWasDown;
    private boolean dpadRightWasDown;
    private boolean touchpadWasTouched;
    private boolean twoControllers;
    private String intakeRobot;
    private int intakeSpeedPercent = 100;
    private boolean launcherWasReady;
    private boolean targetLockRumble;
    private boolean precisionDriving;
    private boolean aiming;
    private Alliance controllerLedAlliance;
    private double leftTrigger;
    private double rightTrigger;
    private static final double INTAKE_TRIGGER_THRESHOLD = 0.5;
    private boolean intakeWasDown;
    private boolean reverseIntakeWasDown;
    private int intakeDirection;

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
        update(gamepad, gamepad);
    }

    public void update(Gamepad gamepad, Gamepad gunner) {
        if (!configurationManaged) {
            updateInit(gamepad);
        }
        precisionDriving = gamepad.left_bumper;
        aiming = gamepad.a;
        leftTrigger = gunner.left_trigger;
        rightTrigger = gunner.right_trigger;
        updateDriveMode(gamepad.back || gamepad.share, gamepad.start || gamepad.options);
        updateLauncherTarget(gunner.dpad_up, gunner.dpad_down);
        if (configurationManaged && intakeRobot != null) {
            updateIntakeSpeed(gunner.dpad_left, gunner.dpad_right);
        }
        if (configurationManaged) {
            updateTouchpadSpeeds(gamepad);
        }

        double forward = -gamepad.left_stick_y;
        double strafe = gamepad.left_stick_x;
        double trackingRotation = tracking.update(gamepad.a);
        double rotate = gamepad.a ? trackingRotation : gamepad.right_stick_x;
        double powerScale = precisionDriving ? bumperSpeed : driveSpeed;
        if (gamepad.a) {
            forward *= powerScale;
            strafe *= powerScale;
            // The selected speed limits translation, not automatic rotation.
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
        updateIntakeToggle();
        mechanisms.update(intakeDirection * intakeSpeedPercent / 100.0, gunner.right_bumper);
        boolean ready = mechanisms.isLauncherReady();
        boolean targetLocked = aiming && tracking.hasTarget();
        if (targetLocked) {
            if (!targetLockRumble) {
                gamepad.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
            }
        } else if (targetLockRumble) {
            gamepad.stopRumble();
        }
        // A shared controller's target lock takes priority over readiness feedback.
        if (gamepad != gunner || (!targetLocked && !targetLockRumble)) {
            if (ready && !launcherWasReady) {
                gunner.rumble(200);
            } else if (!ready && launcherWasReady) {
                gunner.stopRumble();
            }
        }
        targetLockRumble = targetLocked;
        launcherWasReady = ready;
    }

    public boolean isFieldCentric() {
        return fieldCentric;
    }

    public void setIntakeProfile(String robot, int defaultSpeedPercent) {
        intakeRobot = java.util.Objects.requireNonNull(robot);
        intakeSpeedPercent = preferences.getIntakeSpeedPercent(robot, defaultSpeedPercent);
    }

    public int getIntakeSpeedPercent() {
        return intakeSpeedPercent;
    }

    public String getIntakeDirection() {
        return intakeDirection > 0 ? "INTAKE" : intakeDirection < 0 ? "REVERSE" : "OFF";
    }

    public void configure(ConfigurationSubsystem configuration, Gamepad gamepad) {
        configure(configuration, gamepad, gamepad);
    }

    public void configure(ConfigurationSubsystem configuration, Gamepad gamepad, Gamepad gunner) {
        if (!configuration.isLocked()) {
            throw new IllegalStateException("Lock configuration before applying it");
        }
        fieldCentric = configuration.isFieldCentric();
        driveSpeed = configuration.getDriveSpeedPercent() / 100.0;
        bumperSpeed = configuration.getBumperSpeedPercent() / 100.0;
        twoControllers = configuration.isTwoControllers();
        tracking.setAlliance(configuration.getAlliance());
        configurationManaged = true;
        dpadUpWasDown = gunner.dpad_up;
        dpadDownWasDown = gunner.dpad_down;
        dpadLeftWasDown = gunner.dpad_left;
        dpadRightWasDown = gunner.dpad_right;
        touchpadWasTouched = TouchpadInput.isTouched(gamepad);
        intakeDirection = 0;
        intakeWasDown = gunner.right_trigger > INTAKE_TRIGGER_THRESHOLD;
        reverseIntakeWasDown = gunner.left_trigger > INTAKE_TRIGGER_THRESHOLD;
        chordArmed = false;
        chordConsumed = false;
        optionsWasDown = false;
        resetPending = false;
    }

    public int getDriveSpeedPercent() {
        return (int) Math.round(driveSpeed * 100);
    }

    public int getBumperSpeedPercent() {
        return (int) Math.round(bumperSpeed * 100);
    }

    private void updateTouchpadSpeeds(Gamepad gamepad) {
        boolean touched = TouchpadInput.isTouched(gamepad);
        if (touched && !touchpadWasTouched) {
            float x = TouchpadInput.x(gamepad);
            float y = TouchpadInput.y(gamepad);
            if (Float.isFinite(x) && Float.isFinite(y)) {
                if (Math.abs(y) > Math.abs(x) && Math.abs(y) >= 0.3f) {
                    adjustDriveSpeeds(y > 0 ? 5 : -5, 0);
                } else if (Math.abs(x) > Math.abs(y) && Math.abs(x) >= 0.3f) {
                    adjustDriveSpeeds(0, x > 0 ? 5 : -5);
                }
            }
        }
        touchpadWasTouched = touched;
    }

    private void adjustDriveSpeeds(int driveDelta, int bumperDelta) {
        driveSpeed = Math.max(0, Math.min(100, getDriveSpeedPercent() + driveDelta)) / 100.0;
        bumperSpeed = Math.max(0, Math.min(100, getBumperSpeedPercent() + bumperDelta)) / 100.0;
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
        targetLockRumble = false;
        precisionDriving = false;
        aiming = false;
        leftTrigger = 0;
        rightTrigger = 0;
        intakeDirection = 0;
        intakeWasDown = false;
        reverseIntakeWasDown = false;
    }

    @Override
    public void publishTelemetry(RobotTelemetry output) {
        output.status("Drive mode", fieldCentric ? "FIELD-CENTRIC" : "ROBOT-CENTRIC",
                RobotTelemetry.Level.INFO);
        output.status("Driving", aiming ? (precisionDriving ? "AIMING (BUMPER)" : "AIMING (NORMAL)")
                        : precisionDriving ? "BUMPER" : "NORMAL",
                aiming || precisionDriving ? RobotTelemetry.Level.WARNING : RobotTelemetry.Level.INFO);
        output.number("Drive power limit", (aiming ? 1 : precisionDriving
                ? bumperSpeed : driveSpeed) * 100, "%");
        output.number("Translation input scale",
                (aiming ? (precisionDriving ? bumperSpeed : driveSpeed) : 1) * 100, "%");
        output.number("Left trigger", leftTrigger, "");
        output.number("Right trigger", rightTrigger, "");
        output.value("Intake toggle", getIntakeDirection());
        output.number("Intake speed", intakeSpeedPercent, "%");
        String mechanismPad = twoControllers ? "Gunner " : "";
        output.line(mechanismPad + "Right/left trigger: toggle intake/reverse; tap again or press both to stop");
        output.line(mechanismPad + "D-pad left/right: intake speed -/+5%; saved automatically");
        output.line("Left bumper: hold for bumper speed");
        output.line(mechanismPad + "D-pad up/down: launcher target +/-" + LauncherSubsystem.TARGET_VELOCITY_STEP
                + " ticks/s (0-" + LauncherSubsystem.MAX_TARGET_VELOCITY + "); saved automatically");
        if (configurationManaged) {
            output.line("Touchpad top/bottom: drive speed +/-5%; right/left: bumper speed +/-5% (this run only)");
        }
        output.line("Share+Options / Back+Start: toggle drive mode (saved automatically)");
        output.line("Options / Start alone: release to reset heading in field-centric mode");
    }

    private void updateIntakeToggle() {
        boolean intakeDown = rightTrigger > INTAKE_TRIGGER_THRESHOLD;
        boolean reverseDown = leftTrigger > INTAKE_TRIGGER_THRESHOLD;
        if (intakeDown && reverseDown) {
            intakeDirection = 0;
        } else if (intakeDown && !intakeWasDown) {
            intakeDirection = intakeDirection == 1 ? 0 : 1;
        } else if (reverseDown && !reverseIntakeWasDown) {
            intakeDirection = intakeDirection == -1 ? 0 : -1;
        }
        intakeWasDown = intakeDown;
        reverseIntakeWasDown = reverseDown;
    }

    private void updateIntakeSpeed(boolean left, boolean right) {
        int direction = left && !right && !dpadLeftWasDown ? -1
                : right && !left && !dpadRightWasDown ? 1 : 0;
        dpadLeftWasDown = left;
        dpadRightWasDown = right;
        int speed = Math.max(0, Math.min(100, intakeSpeedPercent + direction * 5));
        if (speed != intakeSpeedPercent) {
            preferences.saveIntakeSpeedPercent(intakeRobot, speed);
            intakeSpeedPercent = speed;
        }
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
