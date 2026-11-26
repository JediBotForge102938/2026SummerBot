package org.firstinspires.ftc.teamcode.control;

import android.content.Context;

import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;

import java.util.prefs.BackingStoreException;
import java.util.prefs.Preferences;

/**
 * Desktop replacement for the Android preference adapter, following the Config override layout.
 */
public class DriveModePreferences {
    private static final String FIELD_CENTRIC = "field_centric";
    private static final String LAUNCHER_TARGET_VELOCITY = "launcher_target_velocity";
    private static final String RED_ALLIANCE = "red_alliance";
    private static final String SCORING_START = "scoring_start";
    private static final String TWO_CONTROLLERS = "two_controllers";
    private final Preferences preferences =
            Preferences.userRoot().node("biobuzz/starterbot/teleop");

    public DriveModePreferences(Context context) {
    }

    public boolean isFieldCentric() {
        return preferences.getBoolean(FIELD_CENTRIC, false);
    }

    public boolean getConfigurationFieldCentric() {
        return preferences.getBoolean(FIELD_CENTRIC, true);
    }

    public boolean getConfigurationRedAlliance() {
        return preferences.getBoolean(RED_ALLIANCE, true);
    }

    public boolean isScoringStart() {
        return preferences.getBoolean(SCORING_START, false);
    }

    public int getDriveSpeedPercent() {
        return readSpeed("drive_speed_percent", 60);
    }

    public int getBumperSpeedPercent() {
        return readSpeed("bumper_speed_percent", 100);
    }

    public boolean isTwoControllers() {
        return preferences.getBoolean(TWO_CONTROLLERS, false);
    }

    public void saveTwoControllers(boolean value) {
        preferences.putBoolean(TWO_CONTROLLERS, value);
        flush();
    }

    public void saveDriveSpeedPercent(int value) {
        saveSpeed("drive_speed_percent", value);
    }

    public void saveBumperSpeedPercent(int value) {
        saveSpeed("bumper_speed_percent", value);
    }

    public int getIntakeSpeedPercent(String robot, int defaultValue) {
        validateSpeed(defaultValue);
        return readSpeed("intake_speed_percent_" + robot, defaultValue);
    }

    public void saveIntakeSpeedPercent(String robot, int value) {
        saveSpeed("intake_speed_percent_" + robot, value);
    }

    private int readSpeed(String key, int defaultValue) {
        int value = preferences.getInt(key, defaultValue);
        validateSpeed(value);
        return value;
    }

    private void saveSpeed(String key, int value) {
        validateSpeed(value);
        preferences.putInt(key, value);
        flush();
    }

    private static void validateSpeed(int value) {
        if (value < 0 || value > 100 || value % 5 != 0) {
            throw new IllegalArgumentException("Drive speed must be 0-100 in increments of 5");
        }
    }

    public void saveConfiguration(boolean redAlliance, boolean fieldCentric, boolean scoringStart) {
        preferences.putBoolean(RED_ALLIANCE, redAlliance);
        preferences.putBoolean(FIELD_CENTRIC, fieldCentric);
        preferences.putBoolean(SCORING_START, scoringStart);
        flush();
    }

    public void saveFieldCentric(boolean fieldCentric) {
        preferences.putBoolean(FIELD_CENTRIC, fieldCentric);
        flush();
    }

    public int getLauncherTargetVelocity() {
        return preferences.getInt(LAUNCHER_TARGET_VELOCITY,
                LauncherSubsystem.DEFAULT_TARGET_VELOCITY);
    }

    public void saveLauncherTargetVelocity(int targetVelocity) {
        preferences.putInt(LAUNCHER_TARGET_VELOCITY, targetVelocity);
        flush();
    }

    private void flush() {
        try {
            preferences.flush();
        } catch (BackingStoreException e) {
            throw new IllegalStateException("Unable to save the simulator TeleOp preferences", e);
        }
    }

    public boolean isRedAlliance() {
        return preferences.getBoolean(RED_ALLIANCE, false);
    }

    public void saveRedAlliance(boolean redAlliance) {
        preferences.putBoolean(RED_ALLIANCE, redAlliance);
        flush();
    }


}
