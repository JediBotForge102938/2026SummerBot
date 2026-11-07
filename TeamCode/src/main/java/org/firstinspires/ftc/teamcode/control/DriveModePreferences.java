package org.firstinspires.ftc.teamcode.control;

import android.content.Context;
import android.content.SharedPreferences;

import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;

public class DriveModePreferences {
    private static final String FIELD_CENTRIC = "field_centric";
    private static final String LAUNCHER_TARGET_VELOCITY = "launcher_target_velocity";
    private static final String RED_ALLIANCE = "red_alliance";
    private static final String SCORING_START = "scoring_start";
    private final SharedPreferences preferences;

    public DriveModePreferences(Context context) {
        preferences = context.getSharedPreferences("biobuzz_starterbot_teleop", Context.MODE_PRIVATE);
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

    public void saveConfiguration(boolean redAlliance, boolean fieldCentric, boolean scoringStart) {
        if (!preferences.edit().putBoolean(RED_ALLIANCE, redAlliance)
                .putBoolean(FIELD_CENTRIC, fieldCentric)
                .putBoolean(SCORING_START, scoringStart).commit()) {
            throw new IllegalStateException("Unable to save robot configuration");
        }
    }

    public void saveFieldCentric(boolean fieldCentric) {
        if (!preferences.edit().putBoolean(FIELD_CENTRIC, fieldCentric).commit()) {
            throw new IllegalStateException("Unable to save the TeleOp drive mode");
        }
    }

    public int getLauncherTargetVelocity() {
        return preferences.getInt(LAUNCHER_TARGET_VELOCITY,
                LauncherSubsystem.DEFAULT_TARGET_VELOCITY);
    }

    public void saveLauncherTargetVelocity(int targetVelocity) {
        if (!preferences.edit().putInt(LAUNCHER_TARGET_VELOCITY, targetVelocity).commit()) {
            throw new IllegalStateException("Unable to save the launcher target velocity");
        }
    }

    public boolean isRedAlliance() {
        return preferences.getBoolean(RED_ALLIANCE, false);
    }

    public void saveRedAlliance(boolean redAlliance) {
        if (!preferences.edit().putBoolean(RED_ALLIANCE, redAlliance).commit()) {
            throw new IllegalStateException("Unable to save the tracking alliance");
        }
    }
}
