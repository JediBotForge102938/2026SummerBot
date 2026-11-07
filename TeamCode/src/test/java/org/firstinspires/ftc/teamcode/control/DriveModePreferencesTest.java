package org.firstinspires.ftc.teamcode.control;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import android.content.Context;
import android.content.SharedPreferences;

import org.junit.Before;
import org.junit.Test;
import org.firstinspires.ftc.teamcode.subsystems.ConfigurationSubsystem;
import org.firstinspires.ftc.teamcode.BioBuzzLimelightClusterAdapter.Alliance;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.HashMap;
import java.util.Map;

public class DriveModePreferencesTest {
    private Context context;
    private SharedPreferences storage;
    private SharedPreferences.Editor editor;
    private DriveModePreferences preferences;

    @Before
    public void setUp() {
        context = mock(Context.class);
        storage = mock(SharedPreferences.class);
        editor = mock(SharedPreferences.Editor.class);
        when(context.getSharedPreferences("biobuzz_starterbot_teleop", Context.MODE_PRIVATE))
                .thenReturn(storage);
        when(storage.edit()).thenReturn(editor);
        when(editor.putBoolean(eq("field_centric"), anyBoolean())).thenReturn(editor);
        when(editor.putBoolean(eq("red_alliance"), anyBoolean())).thenReturn(editor);
        when(editor.putBoolean(eq("scoring_start"), anyBoolean())).thenReturn(editor);
        when(editor.putInt(eq("launcher_target_velocity"), anyInt())).thenReturn(editor);
        when(storage.getInt("launcher_target_velocity", 1250)).thenReturn(1250);
        when(editor.commit()).thenReturn(true);
        preferences = new DriveModePreferences(context);
    }

    @Test
    public void configurationDefaultsAndAtomicSaveFailureAreExplicit() {
        when(storage.getBoolean("red_alliance", true)).thenReturn(true);
        when(storage.getBoolean("field_centric", true)).thenReturn(true);
        assertTrue(preferences.getConfigurationRedAlliance());
        assertTrue(preferences.getConfigurationFieldCentric());
        assertFalse(preferences.isScoringStart());
        when(editor.commit()).thenReturn(false);
        assertThrows(IllegalStateException.class,
                () -> preferences.saveConfiguration(false, false, true));
        verify(editor).putBoolean("red_alliance", false);
        verify(editor).putBoolean("field_centric", false);
        verify(editor).putBoolean("scoring_start", true);
        verify(editor).commit();
    }

    @Test
    public void menuSelectionsRestoreAcrossInstancesAndRunningDriveModeChanges() {
        Map<String, Boolean> saved = new HashMap<>();
        when(storage.getBoolean(anyString(), anyBoolean())).thenAnswer(
                invocation -> saved.getOrDefault(invocation.getArgument(0),
                        invocation.getArgument(1)));
        when(editor.putBoolean(anyString(), anyBoolean())).thenAnswer(invocation -> {
            saved.put(invocation.getArgument(0), invocation.getArgument(1));
            return editor;
        });
        ConfigurationSubsystem first = new ConfigurationSubsystem();
        Gamepad gamepad = mock(Gamepad.class);
        first.init(mock(Telemetry.class), gamepad, preferences);
        assertEquals(Alliance.RED, first.getAlliance());
        assertTrue(first.isFieldCentric());
        for (int row = 0; row < 3; row++) {
            gamepad.dpad_right = true;
            first.updateInit(gamepad);
            first.updateInit(gamepad);
            gamepad.dpad_right = false;
            first.updateInit(gamepad);
            gamepad.dpad_down = true;
            first.updateInit(gamepad);
            gamepad.dpad_down = false;
            first.updateInit(gamepad);
        }
        verify(editor, times(3)).commit();
        ConfigurationSubsystem next = new ConfigurationSubsystem();
        next.init(mock(Telemetry.class), gamepad, new DriveModePreferences(context));
        assertEquals(Alliance.BLUE, next.getAlliance());
        assertFalse(next.isFieldCentric());
        assertEquals(ConfigurationSubsystem.StartingSpot.SCORING, next.getStartingSpot());
        verify(gamepad, times(2)).setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
        next.lock();
        next.lock();
        verify(editor, times(4)).commit();
        preferences.saveFieldCentric(true);
        ConfigurationSubsystem third = new ConfigurationSubsystem();
        third.init(mock(Telemetry.class), gamepad, new DriveModePreferences(context));
        assertTrue(third.isFieldCentric());
        assertEquals(Alliance.BLUE, third.getAlliance());
        assertEquals(ConfigurationSubsystem.StartingSpot.SCORING, third.getStartingSpot());
    }

    @Test
    public void failedMenuSaveDoesNotChangeSelectionOrLock() {
        when(storage.getBoolean("red_alliance", true)).thenReturn(true);
        when(storage.getBoolean("field_centric", true)).thenReturn(true);
        ConfigurationSubsystem configuration = new ConfigurationSubsystem();
        Gamepad gamepad = mock(Gamepad.class);
        configuration.init(mock(Telemetry.class), gamepad, preferences);
        clearInvocations(gamepad);
        when(editor.commit()).thenReturn(false);
        gamepad.dpad_right = true;
        assertThrows(IllegalStateException.class, () -> configuration.updateInit(gamepad));
        assertEquals(Alliance.RED, configuration.getAlliance());
        verify(gamepad, never()).setLedColor(anyDouble(), anyDouble(), anyDouble(), anyInt());
        assertThrows(IllegalStateException.class, configuration::lock);
        assertFalse(configuration.isLocked());
    }

    @Test
    public void defaultsToRobotCentric() {
        assertFalse(preferences.isFieldCentric());
        verify(storage).getBoolean("field_centric", false);
    }

    @Test
    public void savesBothModesAndRestoresAcrossInstances() {
        doAnswer(invocation -> {
            boolean value = invocation.getArgument(1);
            when(storage.getBoolean("field_centric", false)).thenReturn(value);
            return editor;
        }).when(editor).putBoolean(eq("field_centric"), anyBoolean());

        preferences.saveFieldCentric(true);
        assertTrue(new DriveModePreferences(context).isFieldCentric());
        preferences.saveFieldCentric(false);
        assertFalse(new DriveModePreferences(context).isFieldCentric());
        verify(editor, times(2)).commit();
    }

    @Test
    public void persistenceFailureIsReported() {
        when(editor.commit()).thenReturn(false);
        assertThrows(IllegalStateException.class, () -> preferences.saveFieldCentric(true));
    }

    @Test
    public void launcherTargetDefaultsToOriginalSpeed() {
        assertEquals(1250, preferences.getLauncherTargetVelocity());
        verify(storage).getInt("launcher_target_velocity", 1250);
    }

    @Test
    public void launcherTargetRestoresAcrossInstancesIncludingZeroAndMaximum() {
        doAnswer(invocation -> {
            int target = invocation.getArgument(1);
            when(storage.getInt("launcher_target_velocity", 1250)).thenReturn(target);
            return editor;
        }).when(editor).putInt(eq("launcher_target_velocity"), anyInt());

        for (int target : new int[] {1500, 0, 6000}) {
            preferences.saveLauncherTargetVelocity(target);
            assertEquals(target, new DriveModePreferences(context).getLauncherTargetVelocity());
        }
        verify(editor, times(3)).commit();
        verify(editor, never()).putBoolean(anyString(), anyBoolean());
    }

    @Test
    public void launcherTargetSaveFailureIsReported() {
        when(editor.commit()).thenReturn(false);
        assertThrows(IllegalStateException.class, () -> preferences.saveLauncherTargetVelocity(1500));
    }

    @Test
    public void allianceDefaultsBlueAndRestoresBothSelections() {
        assertFalse(preferences.isRedAlliance());
        doAnswer(invocation -> {
            boolean red = invocation.getArgument(1);
            when(storage.getBoolean("red_alliance", false)).thenReturn(red);
            return editor;
        }).when(editor).putBoolean(eq("red_alliance"), anyBoolean());
        preferences.saveRedAlliance(true);
        assertTrue(new DriveModePreferences(context).isRedAlliance());
        preferences.saveRedAlliance(false);
        assertFalse(new DriveModePreferences(context).isRedAlliance());
        verify(editor, times(2)).commit();
    }

    @Test
    public void allianceSaveFailureIsExplicit() {
        when(editor.commit()).thenReturn(false);
        assertThrows(IllegalStateException.class, () -> preferences.saveRedAlliance(true));
    }
}
