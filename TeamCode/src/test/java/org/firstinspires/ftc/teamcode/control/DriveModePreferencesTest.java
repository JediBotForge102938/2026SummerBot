package org.firstinspires.ftc.teamcode.control;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import android.content.Context;
import android.content.SharedPreferences;

import org.junit.Before;
import org.junit.Test;

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
        when(editor.putInt(eq("launcher_target_velocity"), anyInt())).thenReturn(editor);
        when(storage.getInt("launcher_target_velocity", 1250)).thenReturn(1250);
        when(editor.commit()).thenReturn(true);
        preferences = new DriveModePreferences(context);
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
