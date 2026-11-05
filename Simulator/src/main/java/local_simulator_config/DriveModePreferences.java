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
    private final Preferences preferences =
            Preferences.userRoot().node("biobuzz/starterbot/teleop");

    public DriveModePreferences(Context context) {
    }

    public boolean isFieldCentric() {
        return preferences.getBoolean(FIELD_CENTRIC, false);
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
