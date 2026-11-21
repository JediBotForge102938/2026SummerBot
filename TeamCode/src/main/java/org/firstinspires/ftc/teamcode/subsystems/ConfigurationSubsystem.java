package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BioBuzzLimelightClusterAdapter.Alliance;
import org.firstinspires.ftc.teamcode.control.DriveModePreferences;

import org.firstinspires.ftc.teamcode.telemetry.RobotTelemetry;
import org.firstinspires.ftc.teamcode.telemetry.TelemetryTable;

/** Owns the pre-start menu; consumers read the locked selections at START. */
public class ConfigurationSubsystem {
    public enum StartingSpot { AUDIENCE, SCORING }
    private enum MenuRow { ALLIANCE, DRIVE_MODE, STARTING_SPOT, DRIVE_SPEED, BUMPER_SPEED, CONTROLLERS }

    private Telemetry telemetry;
    private String robotProfileName;
    private DriveModePreferences preferences;
    private MenuRow row;
    private Alliance alliance;
    private boolean fieldCentric;
    private StartingSpot startingSpot;
    private boolean speedSettingsEnabled;
    private int driveSpeedPercent;
    private int bumperSpeedPercent;
    private boolean twoControllers;
    private boolean locked;
    private final MenuInput firstInput = new MenuInput();
    private final MenuInput secondInput = new MenuInput();

    private static class MenuInput {
        private boolean up;
        private boolean down;
        private boolean left;
        private boolean right;

        void reset() {
            up = down = left = right = false;
        }

        void remember(Gamepad gamepad) {
            up = gamepad != null && gamepad.dpad_up;
            down = gamepad != null && gamepad.dpad_down;
            left = gamepad != null && gamepad.dpad_left;
            right = gamepad != null && gamepad.dpad_right;
        }
    }

    public void init(Telemetry telemetry, Gamepad gamepad, DriveModePreferences preferences) {
        init(telemetry, gamepad, preferences, false);
    }

    public void init(Telemetry telemetry, Gamepad gamepad, DriveModePreferences preferences,
            boolean speedSettingsEnabled) {
        this.telemetry = telemetry;
        this.preferences = preferences;
        robotProfileName = null;
        row = MenuRow.ALLIANCE;
        alliance = preferences.getConfigurationRedAlliance() ? Alliance.RED : Alliance.BLUE;
        fieldCentric = preferences.getConfigurationFieldCentric();
        startingSpot = preferences.isScoringStart() ? StartingSpot.SCORING : StartingSpot.AUDIENCE;
        this.speedSettingsEnabled = speedSettingsEnabled;
        twoControllers = false;
        if (speedSettingsEnabled) {
            driveSpeedPercent = preferences.getDriveSpeedPercent();
            bumperSpeedPercent = preferences.getBumperSpeedPercent();
            twoControllers = preferences.isTwoControllers();
        }
        locked = false;
        firstInput.reset();
        secondInput.reset();
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        telemetry.setAutoClear(false);
        updateAllianceLight(gamepad);
    }

    public void updateInit(Gamepad gamepad) {
        updateInit(gamepad, null);
    }

    public void updateInit(Gamepad gamepad, Gamepad secondGamepad) {
        boolean up = (gamepad.dpad_up && !firstInput.up && !gamepad.dpad_down)
                || (secondGamepad != null && secondGamepad.dpad_up && !secondInput.up
                        && !secondGamepad.dpad_down);
        boolean down = (gamepad.dpad_down && !firstInput.down && !gamepad.dpad_up)
                || (secondGamepad != null && secondGamepad.dpad_down && !secondInput.down
                        && !secondGamepad.dpad_up);
        boolean left = (gamepad.dpad_left && !firstInput.left && !gamepad.dpad_right)
                || (secondGamepad != null && secondGamepad.dpad_left && !secondInput.left
                        && !secondGamepad.dpad_right);
        boolean right = (gamepad.dpad_right && !firstInput.right && !gamepad.dpad_left)
                || (secondGamepad != null && secondGamepad.dpad_right && !secondInput.right
                        && !secondGamepad.dpad_left);
        if (!locked) {
            MenuRow[] rows = MenuRow.values();
            int rowCount = speedSettingsEnabled ? rows.length : MenuRow.STARTING_SPOT.ordinal() + 1;
            if (up && !down) {
                row = rows[(row.ordinal() + rowCount - 1) % rowCount];
            } else if (down && !up) {
                row = rows[(row.ordinal() + 1) % rowCount];
            }
            if (left != right) {
                switch (row) {
                    case ALLIANCE:
                        preferences.saveConfiguration(alliance != Alliance.RED, fieldCentric,
                                startingSpot == StartingSpot.SCORING);
                        alliance = alliance == Alliance.RED ? Alliance.BLUE : Alliance.RED;
                        updateAllianceLight(gamepad);
                        break;
                    case DRIVE_MODE:
                        preferences.saveConfiguration(alliance == Alliance.RED, !fieldCentric,
                                startingSpot == StartingSpot.SCORING);
                        fieldCentric = !fieldCentric;
                        break;
                    case STARTING_SPOT:
                        preferences.saveConfiguration(alliance == Alliance.RED, fieldCentric,
                                startingSpot != StartingSpot.SCORING);
                        startingSpot = startingSpot == StartingSpot.AUDIENCE
                                ? StartingSpot.SCORING : StartingSpot.AUDIENCE;
                        break;
                    case DRIVE_SPEED: {
                        int value = adjustSpeed(driveSpeedPercent, right);
                        if (value != driveSpeedPercent) {
                            preferences.saveDriveSpeedPercent(value);
                            driveSpeedPercent = value;
                        }
                        break;
                    }
                    case BUMPER_SPEED: {
                        int value = adjustSpeed(bumperSpeedPercent, right);
                        if (value != bumperSpeedPercent) {
                            preferences.saveBumperSpeedPercent(value);
                            bumperSpeedPercent = value;
                        }
                        break;
                    }
                    case CONTROLLERS:
                        preferences.saveTwoControllers(!twoControllers);
                        twoControllers = !twoControllers;
                        break;
                }
            }
        }
        firstInput.remember(gamepad);
        secondInput.remember(secondGamepad);
    }

    public void lock() {
        if (!locked) {
            preferences.saveConfiguration(alliance == Alliance.RED, fieldCentric,
                    startingSpot == StartingSpot.SCORING);
        }
        locked = true;
    }

    public boolean isLocked() {
        return locked;
    }

    public Alliance getAlliance() {
        return alliance;
    }

    public boolean isFieldCentric() {
        return fieldCentric;
    }

    public StartingSpot getStartingSpot() {
        return startingSpot;
    }

    public int getDriveSpeedPercent() {
        return driveSpeedPercent;
    }

    public int getBumperSpeedPercent() {
        return bumperSpeedPercent;
    }

    public boolean isTwoControllers() {
        return twoControllers;
    }

    private static int adjustSpeed(int value, boolean increase) {
        return Math.max(0, Math.min(100, value + (increase ? 5 : -5)));
    }

    public double getStartingHeadingDegrees() {
        // Field zero is Red-to-Blue. Rear starts face the audience; audience starts face rear.
        return startingSpot == StartingSpot.SCORING ? 270 : 90;
    }

    public double getFieldForwardHeadingDegrees() {
        return alliance == Alliance.RED ? 0 : 180;
    }

    /** Writes the configuration portion of a caller-owned SDK telemetry frame. */
    public void publish() {
        publish(fieldCentric);
    }

    public void publish(boolean currentFieldCentric) {
        publish(currentFieldCentric, driveSpeedPercent, bumperSpeedPercent);
    }

    public void publish(boolean currentFieldCentric, int currentDriveSpeed, int currentBumperSpeed) {
        TelemetryTable.heading(telemetry, "Configuration");
        if (robotProfileName != null) {
            telemetry.addData("Robot profile", robotProfileName);
        }
        menuLine(MenuRow.ALLIANCE, "Alliance", alliance == Alliance.RED ? "Red" : "Blue");
        menuLine(MenuRow.DRIVE_MODE, "Drive mode",
                currentFieldCentric ? "Field-centric" : "Robot-centric");
        menuLine(MenuRow.STARTING_SPOT, "Starting spot",
                startingSpot == StartingSpot.SCORING ? "Scoring" : "Audience");
        if (speedSettingsEnabled) {
            menuLine(MenuRow.DRIVE_SPEED, "Drive speed", currentDriveSpeed + "%");
            menuLine(MenuRow.BUMPER_SPEED, "Bumper speed", currentBumperSpeed + "%");
            menuLine(MenuRow.CONTROLLERS, "Controllers", twoControllers ? "2" : "1");
        }
    }

    public void setRobotProfileName(String name) {
        robotProfileName = name;
    }

    private void menuLine(MenuRow menuRow, String label, String value) {
        TelemetryTable.row(telemetry, label, value, !locked && row == menuRow,
                menuRow == MenuRow.ALLIANCE ? alliance == Alliance.RED
                        ? RobotTelemetry.Color.RED : RobotTelemetry.Color.BLUE : null);
    }

    private void updateAllianceLight(Gamepad gamepad) {
        gamepad.setLedColor(alliance == Alliance.RED ? 1 : 0, 0,
                alliance == Alliance.BLUE ? 1 : 0, Gamepad.LED_DURATION_CONTINUOUS);
    }
}
