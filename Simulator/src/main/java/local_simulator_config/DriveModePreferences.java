package org.firstinspires.ftc.teamcode.control;

import android.content.Context;

import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FieldMap;

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
    private static final String NAV_HOME_X = "navigation_home_x";
    private static final String NAV_HOME_Y = "navigation_home_y";
    private static final String NAV_HOME_HEADING = "navigation_home_heading";
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

    public FieldMap.Pose getNavigationHome() {
        double x = preferences.getDouble(NAV_HOME_X, Double.NaN);
        double y = preferences.getDouble(NAV_HOME_Y, Double.NaN);
        double heading = preferences.getDouble(NAV_HOME_HEADING, Double.NaN);
        return Double.isFinite(x) && Double.isFinite(y) && Double.isFinite(heading)
                ? new FieldMap.Pose(x, y, heading) : null;
    }

    public void saveNavigationHome(FieldMap.Pose pose) {
        if (pose == null) throw new IllegalArgumentException("Saved home pose is required");
        preferences.putDouble(NAV_HOME_X, pose.x);
        preferences.putDouble(NAV_HOME_Y, pose.y);
        preferences.putDouble(NAV_HOME_HEADING, pose.headingDegrees);
        flush();
    }
}
