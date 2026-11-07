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
    private enum MenuRow { ALLIANCE, DRIVE_MODE, STARTING_SPOT }

    private Telemetry telemetry;
    private String robotProfileName;
    private DriveModePreferences preferences;
    private MenuRow row;
    private Alliance alliance;
    private boolean fieldCentric;
    private StartingSpot startingSpot;
    private boolean locked;
    private boolean previousUp;
    private boolean previousDown;
    private boolean previousLeft;
    private boolean previousRight;

    public void init(Telemetry telemetry, Gamepad gamepad, DriveModePreferences preferences) {
        this.telemetry = telemetry;
        this.preferences = preferences;
        robotProfileName = null;
        row = MenuRow.ALLIANCE;
        alliance = preferences.getConfigurationRedAlliance() ? Alliance.RED : Alliance.BLUE;
        fieldCentric = preferences.getConfigurationFieldCentric();
        startingSpot = preferences.isScoringStart() ? StartingSpot.SCORING : StartingSpot.AUDIENCE;
        locked = false;
        previousUp = false;
        previousDown = false;
        previousLeft = false;
        previousRight = false;
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        telemetry.setAutoClear(false);
        updateAllianceLight(gamepad);
    }

    public void updateInit(Gamepad gamepad) {
        boolean up = gamepad.dpad_up;
        boolean down = gamepad.dpad_down;
        boolean left = gamepad.dpad_left;
        boolean right = gamepad.dpad_right;
        if (!locked) {
            MenuRow[] rows = MenuRow.values();
            if (up && !previousUp && !down) {
                row = rows[(row.ordinal() + rows.length - 1) % rows.length];
            } else if (down && !previousDown && !up) {
                row = rows[(row.ordinal() + 1) % rows.length];
            }
            if ((left && !previousLeft && !right)
                    || (right && !previousRight && !left)) {
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
                }
            }
        }
        previousUp = up;
        previousDown = down;
        previousLeft = left;
        previousRight = right;
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
        TelemetryTable.heading(telemetry, "Configuration");
        if (robotProfileName != null) {
            TelemetryTable.row(telemetry, "Robot profile", robotProfileName);
        }
        menuLine(MenuRow.ALLIANCE, "Alliance", alliance == Alliance.RED ? "Red" : "Blue");
        menuLine(MenuRow.DRIVE_MODE, "Drive mode",
                currentFieldCentric ? "Field-centric" : "Robot-centric");
        menuLine(MenuRow.STARTING_SPOT, "Starting spot",
                startingSpot == StartingSpot.SCORING ? "Scoring" : "Audience");
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
