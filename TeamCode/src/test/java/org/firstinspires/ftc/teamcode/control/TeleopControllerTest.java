package org.firstinspires.ftc.teamcode.control;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.mockito.Mockito.*;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.BioBuzzLimelightClusterAdapter.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ConfigurationSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.telemetry.RobotTelemetry;
import org.junit.Before;
import org.junit.Test;

public class TeleopControllerTest {
    private DriveSubsystem drive;
    private MechanismCoordinator mechanisms;
    private DriveModePreferences preferences;
    private Gamepad gamepad;
    private TeleopController controller;
    private HiveTrackingController tracking;

    @Before
    public void setUp() {
        drive = mock(DriveSubsystem.class);
        mechanisms = mock(MechanismCoordinator.class);
        preferences = mock(DriveModePreferences.class);
        when(preferences.getLauncherTargetVelocity()).thenReturn(
                LauncherSubsystem.DEFAULT_TARGET_VELOCITY);
        doAnswer(invocation -> {
            int target = invocation.getArgument(0);
            when(mechanisms.getLauncherTargetVelocity()).thenReturn(target);
            return null;
        }).when(mechanisms).setLauncherTargetVelocity(anyInt());
        gamepad = mock(Gamepad.class);
        tracking = mock(HiveTrackingController.class);
        when(tracking.getAlliance()).thenReturn(Alliance.BLUE);
        controller = new TeleopController(drive, mechanisms, preferences, tracking);
        controller.update(gamepad);
        clearInvocations(drive, mechanisms, preferences);
    }

    @Test
    public void lockedConfigurationSetsModeAndAllianceAndSuppressesHeldInitInputs() {
        ConfigurationSubsystem configuration = mock(ConfigurationSubsystem.class);
        when(configuration.isLocked()).thenReturn(true);
        when(configuration.isFieldCentric()).thenReturn(true);
        when(configuration.getDriveSpeedPercent()).thenReturn(100);
        when(configuration.getBumperSpeedPercent()).thenReturn(35);
        when(configuration.getAlliance()).thenReturn(Alliance.RED);
        gamepad.dpad_up = true;
        gamepad.dpad_left = true;
        controller.configure(configuration, gamepad);
        verify(tracking).setAlliance(Alliance.RED);
        clearInvocations(tracking, gamepad);
        controller.update(gamepad);
        assertTrue(controller.isFieldCentric());
        verify(drive).driveFieldRelative(-0.0, 0, 0, 1);
        verify(tracking, never()).selectAlliance(anyBoolean(), anyBoolean());
        verify(gamepad, never()).setLedColor(anyDouble(), anyDouble(), anyDouble(), anyInt());
        verifyNoInteractions(preferences);
        gamepad.dpad_up = false;
        controller.update(gamepad);
        gamepad.dpad_up = true;
        controller.update(gamepad);
        verify(preferences).saveLauncherTargetVelocity(
                LauncherSubsystem.DEFAULT_TARGET_VELOCITY + LauncherSubsystem.TARGET_VELOCITY_STEP);
    }

    @Test
    public void twoControllersSeparateControlsAndKeepDriveAdjustmentsVolatile() {
        ConfigurationSubsystem config = mock(ConfigurationSubsystem.class);
        when(config.isLocked()).thenReturn(true);
        when(config.isTwoControllers()).thenReturn(true);
        when(config.getDriveSpeedPercent()).thenReturn(95);
        when(config.getBumperSpeedPercent()).thenReturn(35);
        when(preferences.getIntakeSpeedPercent("competitionbot", 75)).thenReturn(75);
        controller.setIntakeProfile("competitionbot", 75);
        Gamepad gunner = mock(Gamepad.class);
        gamepad.dpad_up = true;
        gunner.dpad_up = true;
        gunner.right_trigger = 1;
        controller.configure(config, gamepad, gunner);
        clearInvocations(preferences, mechanisms, drive);
        controller.update(gamepad, gunner);
        assertTrue(controller.getDriveSpeedPercent() == 95);
        verify(mechanisms).update(0, false);
        verify(preferences, never()).saveLauncherTargetVelocity(anyInt());

        gamepad.dpad_up = false;
        gunner.dpad_up = false;
        gunner.right_trigger = 0;
        controller.update(gamepad, gunner);
        gamepad.dpad_up = true;
        gamepad.dpad_right = true;
        gamepad.right_bumper = true;
        gamepad.right_trigger = 1;
        gunner.dpad_up = true;
        gunner.dpad_right = true;
        gunner.right_trigger = 1;
        gunner.right_bumper = true;
        controller.update(gamepad, gunner);
        controller.update(gamepad, gunner);
        assertTrue(controller.getDriveSpeedPercent() == 95);
        assertTrue(controller.getBumperSpeedPercent() == 35);
        gamepad.touchpad_finger_1 = true;
        gamepad.touchpad_finger_1_y = 0.8f;
        controller.update(gamepad, gunner);
        gamepad.touchpad_finger_1 = false;
        controller.update(gamepad, gunner);
        gamepad.touchpad_finger_1 = true;
        gamepad.touchpad_finger_1_y = 0;
        gamepad.touchpad_finger_1_x = 0.8f;
        controller.update(gamepad, gunner);
        assertTrue(controller.getDriveSpeedPercent() == 100);
        assertTrue(controller.getBumperSpeedPercent() == 40);
        verify(mechanisms, times(5)).update(0.8, true);
        verify(preferences).saveIntakeSpeedPercent("competitionbot", 80);
        verify(preferences).saveLauncherTargetVelocity(
                LauncherSubsystem.DEFAULT_TARGET_VELOCITY + LauncherSubsystem.TARGET_VELOCITY_STEP);
        verify(preferences, never()).saveDriveSpeedPercent(anyInt());
        verify(preferences, never()).saveBumperSpeedPercent(anyInt());
        gamepad.left_bumper = true;
        gamepad.left_stick_y = -1;
        controller.update(gamepad, gunner);
        verify(drive).driveRobotRelative(1, 0, 0, 0.4);
        gamepad.a = true;
        controller.update(gamepad, gunner);
        verify(drive).driveRobotRelative(0.4, 0, 0, 1, true);
        for (int i = 0; i < 21; i++) {
            gamepad.touchpad_finger_1 = false;
            controller.update(gamepad, gunner);
            gamepad.touchpad_finger_1 = true;
            gamepad.touchpad_finger_1_x = 0;
            gamepad.touchpad_finger_1_y = -0.8f;
            controller.update(gamepad, gunner);
            gamepad.touchpad_finger_1 = false;
            controller.update(gamepad, gunner);
            gamepad.touchpad_finger_1 = true;
            gamepad.touchpad_finger_1_x = -0.8f;
            gamepad.touchpad_finger_1_y = 0;
            controller.update(gamepad, gunner);
        }
        assertTrue(controller.getDriveSpeedPercent() == 0);
        assertTrue(controller.getBumperSpeedPercent() == 0);
    }

    @Test
    public void twoControllerDriverButtonsDoNotTakeOverGunnerAndDriveModeStaysOnDriver() {
        ConfigurationSubsystem config = mock(ConfigurationSubsystem.class);
        when(config.isLocked()).thenReturn(true);
        when(config.isTwoControllers()).thenReturn(true);
        when(config.getDriveSpeedPercent()).thenReturn(50);
        when(config.getBumperSpeedPercent()).thenReturn(35);
        Gamepad gunner = mock(Gamepad.class);
        controller.configure(config, gamepad, gunner);
        clearInvocations(preferences, mechanisms, drive);
        gamepad.right_bumper = true;
        gamepad.right_trigger = 1;
        gamepad.dpad_up = true;
        gamepad.dpad_down = true;
        gamepad.dpad_left = true;
        gamepad.dpad_right = true;
        controller.update(gamepad, gunner);
        verify(mechanisms).update(0, false);
        assertTrue(controller.getDriveSpeedPercent() == 50);
        assertTrue(controller.getBumperSpeedPercent() == 35);
        verify(preferences, never()).saveLauncherTargetVelocity(anyInt());
        gamepad.share = true;
        gamepad.options = true;
        controller.update(gamepad, gunner);
        assertTrue(controller.isFieldCentric());
        verify(preferences).saveFieldCentric(true);
        gamepad.share = false;
        gamepad.options = false;
        controller.update(gamepad, gunner);
        gamepad.options = true;
        controller.update(gamepad, gunner);
        gamepad.options = false;
        controller.update(gamepad, gunner);
        verify(drive).resetHeading();
    }

    @Test
    public void gunnerTriggersToggleIntakeAndDriverTriggersDoNot() {
        ConfigurationSubsystem config = mock(ConfigurationSubsystem.class);
        when(config.isLocked()).thenReturn(true);
        when(config.isTwoControllers()).thenReturn(true);
        Gamepad gunner = mock(Gamepad.class);
        controller.configure(config, gamepad, gunner);
        clearInvocations(mechanisms);

        gamepad.right_trigger = 1;
        controller.update(gamepad, gunner);
        verify(mechanisms).update(0, false);
        gunner.right_trigger = 1;
        controller.update(gamepad, gunner);
        verify(mechanisms).update(1, false);
        assertTrue(controller.getIntakeDirection().equals("INTAKE"));
        gunner.right_trigger = 0;
        gunner.left_trigger = 1;
        controller.update(gamepad, gunner);
        verify(mechanisms).update(-1, false);
        assertTrue(controller.getIntakeDirection().equals("REVERSE"));
        gunner.right_trigger = 1;
        controller.update(gamepad, gunner);
        verify(mechanisms, times(2)).update(0, false);
        assertTrue(controller.getIntakeDirection().equals("OFF"));
    }

    @Test
    public void gunnerReadinessRumbleIsIndependentOfDriverTargetLock() {
        ConfigurationSubsystem config = mock(ConfigurationSubsystem.class);
        when(config.isLocked()).thenReturn(true);
        when(config.isTwoControllers()).thenReturn(true);
        Gamepad gunner = mock(Gamepad.class);
        controller.configure(config, gamepad, gunner);
        clearInvocations(gamepad, gunner);
        gamepad.a = true;
        when(tracking.hasTarget()).thenReturn(true);
        when(mechanisms.isLauncherReady()).thenReturn(false, true, false);
        controller.update(gamepad, gunner);
        controller.update(gamepad, gunner);
        controller.update(gamepad, gunner);
        verify(gamepad).rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
        verify(gamepad, never()).rumble(200);
        verify(gamepad, never()).stopRumble();
        verify(gunner).rumble(200);
        verify(gunner).stopRumble();
        when(tracking.hasTarget()).thenReturn(false);
        controller.update(gamepad, gunner);
        verify(gamepad).stopRumble();
    }

    @Test
    public void singleControllerTouchpadAdjustsSpeedsOncePerTapWithoutChangingDpad() {
        ConfigurationSubsystem config = mock(ConfigurationSubsystem.class);
        when(config.isLocked()).thenReturn(true);
        when(config.getDriveSpeedPercent()).thenReturn(95);
        when(config.getBumperSpeedPercent()).thenReturn(35);
        gamepad.touchpad_finger_1 = true;
        gamepad.touchpad_finger_1_y = 0.8f;
        controller.configure(config, gamepad);
        clearInvocations(preferences);
        controller.update(gamepad);
        assertTrue(controller.getDriveSpeedPercent() == 95);
        gamepad.touchpad_finger_1 = false;
        controller.update(gamepad);
        gamepad.touchpad_finger_1 = true;
        gamepad.dpad_up = true;
        controller.update(gamepad);
        controller.update(gamepad);
        assertTrue(controller.getDriveSpeedPercent() == 100);
        verify(preferences).saveLauncherTargetVelocity(
                LauncherSubsystem.DEFAULT_TARGET_VELOCITY + LauncherSubsystem.TARGET_VELOCITY_STEP);
        gamepad.touchpad_finger_1 = false;
        gamepad.dpad_up = false;
        controller.update(gamepad);
        gamepad.touchpad_finger_1 = true;
        gamepad.touchpad_finger_1_y = -0.8f;
        controller.update(gamepad);
        assertTrue(controller.getDriveSpeedPercent() == 95);
        gamepad.touchpad_finger_1 = false;
        controller.update(gamepad);
        gamepad.touchpad_finger_1 = true;
        gamepad.touchpad_finger_1_x = 0.8f;
        gamepad.touchpad_finger_1_y = 0;
        controller.update(gamepad);
        assertTrue(controller.getBumperSpeedPercent() == 40);
        gamepad.touchpad_finger_1 = false;
        controller.update(gamepad);
        gamepad.touchpad_finger_1 = true;
        gamepad.touchpad_finger_1_x = -0.8f;
        controller.update(gamepad);
        assertTrue(controller.getBumperSpeedPercent() == 35);
        gamepad.touchpad_finger_1 = false;
        controller.update(gamepad);
        gamepad.touchpad_finger_1 = true;
        gamepad.touchpad_finger_1_x = 0;
        gamepad.touchpad_finger_1_y = 0.1f;
        controller.update(gamepad);
        assertTrue(controller.getDriveSpeedPercent() == 95);
        assertTrue(controller.getBumperSpeedPercent() == 35);
        verify(preferences, never()).saveDriveSpeedPercent(anyInt());
        verify(preferences, never()).saveBumperSpeedPercent(anyInt());
    }

    @Test
    public void twoControllerSpeedUsesOnlyDriverTouchpad() {
        ConfigurationSubsystem config = mock(ConfigurationSubsystem.class);
        when(config.isLocked()).thenReturn(true);
        when(config.isTwoControllers()).thenReturn(true);
        when(config.getDriveSpeedPercent()).thenReturn(50);
        when(config.getBumperSpeedPercent()).thenReturn(35);
        Gamepad gunner = mock(Gamepad.class);
        gamepad.touchpad_finger_1 = true;
        gamepad.touchpad_finger_1_y = 1;
        controller.configure(config, gamepad, gunner);
        controller.update(gamepad, gunner);
        assertTrue(controller.getDriveSpeedPercent() == 50);
        gamepad.touchpad_finger_1 = false;
        gunner.touchpad_finger_1 = true;
        gunner.touchpad_finger_1_y = 1;
        gamepad.dpad_up = true;
        controller.update(gamepad, gunner);
        assertTrue(controller.getDriveSpeedPercent() == 50);
        gamepad.touchpad_finger_1 = true;
        controller.update(gamepad, gunner);
        controller.update(gamepad, gunner);
        assertTrue(controller.getDriveSpeedPercent() == 55);
        assertTrue(controller.getBumperSpeedPercent() == 35);
    }

    @Test
    public void mapsExistingInputsAndDefaultsToRobotCentric() {
        gamepad.left_stick_y = -0.5f;
        gamepad.left_stick_x = 0.25f;
        gamepad.right_stick_x = -0.75f;
        gamepad.right_trigger = 0.75f;
        gamepad.left_trigger = 0.25f;
        gamepad.right_bumper = true;

        controller.update(gamepad);

        assertFalse(controller.isFieldCentric());
        verify(drive).driveRobotRelative(0.5, 0.25, -0.75, 1);
        verify(mechanisms).update(1, true);
        verifyNoInteractions(preferences);
    }

    @Test public void intakeTriggersToggleOnceSwitchDirectionAndStopTogether() {
        gamepad.right_trigger = 1;
        controller.update(gamepad);
        controller.update(gamepad);
        gamepad.right_trigger = 0;
        controller.update(gamepad);
        verify(mechanisms, times(3)).update(1, false);
        clearInvocations(mechanisms);
        gamepad.right_trigger = 1;
        controller.update(gamepad);
        verify(mechanisms).update(0, false);
        gamepad.right_trigger = 0;
        gamepad.left_trigger = 1;
        controller.update(gamepad);
        controller.update(gamepad);
        gamepad.left_trigger = 0;
        controller.update(gamepad);
        verify(mechanisms, times(3)).update(-1, false);
        clearInvocations(mechanisms);
        gamepad.left_trigger = 1;
        controller.update(gamepad);
        verify(mechanisms).update(0, false);
        gamepad.left_trigger = 0;
        gamepad.right_trigger = 1;
        controller.update(gamepad);
        gamepad.right_trigger = 0;
        gamepad.left_trigger = 1;
        controller.update(gamepad);
        clearInvocations(mechanisms);
        gamepad.right_trigger = 1;
        controller.update(gamepad);
        gamepad.left_trigger = 0;
        controller.update(gamepad);
        verify(mechanisms, times(2)).update(0, false);
    }

    @Test public void intakeSpeedAdjustsOnPressAndSuppressesHeldInitButtons() {
        when(preferences.getIntakeSpeedPercent("competitionbot", 75)).thenReturn(75);
        controller.setIntakeProfile("competitionbot", 75);
        ConfigurationSubsystem configuration = mock(ConfigurationSubsystem.class);
        when(configuration.isLocked()).thenReturn(true);
        gamepad.dpad_right = true;
        controller.configure(configuration, gamepad);
        gamepad.right_trigger = 1;
        controller.update(gamepad);
        verify(mechanisms).update(0.75, false);
        verify(preferences, never()).saveIntakeSpeedPercent(anyString(), anyInt());
        gamepad.dpad_right = false;
        controller.update(gamepad);
        gamepad.dpad_right = true;
        controller.update(gamepad);
        controller.update(gamepad);
        verify(preferences).saveIntakeSpeedPercent("competitionbot", 80);
        verify(mechanisms, times(2)).update(0.8, false);
        gamepad.dpad_right = false;
        gamepad.dpad_left = true;
        gamepad.right_trigger = 0;
        gamepad.left_trigger = 1;
        controller.update(gamepad);
        verify(mechanisms).update(-0.75, false);
    }

    @Test public void failedIntakeSpeedSaveDoesNotChangeSpeed() {
        when(preferences.getIntakeSpeedPercent("starterbot", 100)).thenReturn(100);
        controller.setIntakeProfile("starterbot", 100);
        ConfigurationSubsystem configuration = mock(ConfigurationSubsystem.class);
        when(configuration.isLocked()).thenReturn(true);
        controller.configure(configuration, gamepad);
        doThrow(new IllegalStateException("Cannot save")).when(preferences)
                .saveIntakeSpeedPercent("starterbot", 95);
        gamepad.dpad_left = true;
        org.junit.Assert.assertThrows(IllegalStateException.class, () -> controller.update(gamepad));
        org.junit.Assert.assertEquals(100, controller.getIntakeSpeedPercent());
    }

    @Test public void configuredHeldTriggersRequireReleaseAndStopClearsToggle() {
        ConfigurationSubsystem configuration = mock(ConfigurationSubsystem.class);
        when(configuration.isLocked()).thenReturn(true);
        when(configuration.getAlliance()).thenReturn(Alliance.RED);
        gamepad.right_trigger = 1;
        controller.configure(configuration, gamepad);
        controller.update(gamepad);
        verify(mechanisms).update(0, false);
        gamepad.right_trigger = 0;
        controller.update(gamepad);
        gamepad.right_trigger = 1;
        controller.update(gamepad);
        verify(mechanisms).update(1, false);
        controller.stop();
        clearInvocations(mechanisms);
        gamepad.right_trigger = 0;
        controller.update(gamepad);
        verify(mechanisms).update(0, false);
    }

    @Test
    public void restoresFieldCentricWithoutResettingHeading() {
        when(preferences.isFieldCentric()).thenReturn(true);
        controller = new TeleopController(drive, mechanisms, preferences, tracking);

        controller.update(gamepad);

        assertTrue(controller.isFieldCentric());
        verify(drive).driveFieldRelative(-0.0, 0, 0, 1);
        verify(drive, never()).resetHeading();
        verify(preferences, never()).saveFieldCentric(anyBoolean());
    }

    @Test
    public void chordTogglesOnceAndRequiresBothButtonsReleasedToRearm() {
        buttons(true, true);
        buttons(true, true);
        buttons(false, true);
        buttons(true, true);
        buttons(true, false);
        buttons(true, true);

        assertTrue(controller.isFieldCentric());
        verify(preferences).saveFieldCentric(true);
        verify(drive, never()).resetHeading();

        buttons(false, false);
        buttons(true, true);
        buttons(false, false);

        assertFalse(controller.isFieldCentric());
        verify(preferences).saveFieldCentric(false);
        verifyNoMoreInteractions(preferences);
        verify(drive, never()).resetHeading();
    }

    @Test
    public void eitherChordButtonOrderDoesNotResetHeading() {
        buttons(false, true);
        buttons(true, true);
        buttons(false, true);
        buttons(false, false);

        buttons(true, false);
        buttons(true, true);
        buttons(true, false);
        buttons(false, false);

        verify(preferences).saveFieldCentric(true);
        verify(preferences).saveFieldCentric(false);
        verify(drive, never()).resetHeading();
    }

    @Test
    public void standaloneOptionsResetsOnceOnReleaseOnlyInFieldCentric() {
        buttons(false, true);
        buttons(false, false);
        verify(drive, never()).resetHeading();

        buttons(true, true);
        buttons(false, false);
        buttons(false, true);
        buttons(false, true);
        verify(drive, never()).resetHeading();

        buttons(false, false);
        buttons(false, false);
        verify(drive).resetHeading();
    }

    @Test
    public void xboxAliasesSupportToggleAndStandaloneReset() {
        gamepad.back = true;
        gamepad.start = true;
        controller.update(gamepad);
        gamepad.back = false;
        gamepad.start = false;
        controller.update(gamepad);
        gamepad.start = true;
        controller.update(gamepad);
        gamepad.start = false;
        controller.update(gamepad);

        assertTrue(controller.isFieldCentric());
        verify(preferences).saveFieldCentric(true);
        verify(drive).resetHeading();
    }

    @Test
    public void startAndOptionsAliasesAloneNeverToggleMode() {
        gamepad.start = true;
        gamepad.options = true;
        controller.update(gamepad);
        gamepad.start = false;
        gamepad.options = false;
        controller.update(gamepad);

        assertFalse(controller.isFieldCentric());
        verifyNoInteractions(preferences);
        verify(drive, never()).resetHeading();
    }

    @Test
    public void oldAButtonDoesNotResetHeading() {
        when(preferences.isFieldCentric()).thenReturn(true);
        controller = new TeleopController(drive, mechanisms, preferences, tracking);
        gamepad.a = true;
        controller.update(gamepad);

        verify(drive, never()).resetHeading();
    }

    @Test
    public void buttonsHeldAtStartupMustBeReleasedFirst() {
        when(preferences.isFieldCentric()).thenReturn(true);
        controller = new TeleopController(drive, mechanisms, preferences, tracking);
        buttons(true, true);
        buttons(false, true);
        buttons(false, false);

        assertTrue(controller.isFieldCentric());
        verify(preferences, never()).saveFieldCentric(anyBoolean());
        verify(drive, never()).resetHeading();

        buttons(true, true);
        assertFalse(controller.isFieldCentric());
    }

    @Test
    public void failedSaveDoesNotClaimModeChanged() {
        doThrow(new IllegalStateException("Cannot save")).when(preferences).saveFieldCentric(true);

        org.junit.Assert.assertThrows(IllegalStateException.class, () -> buttons(true, true));

        assertFalse(controller.isFieldCentric());
    }

    @Test
    public void stopClearsAllOutputsWithoutChangingSavedMode() {
        controller.stop();

        verify(drive).driveRobotRelative(0, 0, 0);
        verify(mechanisms).stop();
        verifyNoInteractions(preferences);
    }

    @Test
    public void restoresSavedLauncherTargetBeforeFirstUpdate() {
        when(preferences.getLauncherTargetVelocity()).thenReturn(1750);
        controller = new TeleopController(drive, mechanisms, preferences, tracking);

        verify(mechanisms).setLauncherTargetVelocity(1750);
        verify(preferences, never()).saveLauncherTargetVelocity(anyInt());
    }

    @Test
    public void dpadAdjustsBy25OnlyOncePerPressAndSavesEachChange() {
        dpad(true, false);
        dpad(true, false);
        verify(preferences).saveLauncherTargetVelocity(1275);
        verify(mechanisms).setLauncherTargetVelocity(1275);

        dpad(false, false);
        dpad(true, false);
        verify(preferences).saveLauncherTargetVelocity(1300);
        verify(mechanisms).setLauncherTargetVelocity(1300);

        dpad(false, true);
        dpad(false, true);
        verify(preferences, times(2)).saveLauncherTargetVelocity(1275);
        verify(mechanisms, times(2)).setLauncherTargetVelocity(1275);
        verify(preferences, times(3)).saveLauncherTargetVelocity(anyInt());
    }

    @Test
    public void simultaneousDpadPressesDoNothingAndMustBeRepressed() {
        dpad(true, true);
        dpad(true, false);
        dpad(false, false);
        verifyNoInteractions(preferences);
        verify(mechanisms, never()).setLauncherTargetVelocity(anyInt());

        dpad(true, false);
        verify(preferences).saveLauncherTargetVelocity(1275);
    }

    @Test
    public void clampsAtBothBoundsAndDoesNotSaveNoOpPresses() {
        when(mechanisms.getLauncherTargetVelocity()).thenReturn(5990);
        dpad(true, false);
        verify(preferences).saveLauncherTargetVelocity(6000);
        dpad(false, false);
        dpad(true, false);
        verify(preferences, times(1)).saveLauncherTargetVelocity(6000);

        when(mechanisms.getLauncherTargetVelocity()).thenReturn(10);
        dpad(false, true);
        verify(preferences).saveLauncherTargetVelocity(0);
        dpad(false, false);
        dpad(false, true);
        verify(preferences, times(1)).saveLauncherTargetVelocity(0);
        verify(preferences, times(2)).saveLauncherTargetVelocity(anyInt());
    }

    @Test
    public void failedSpeedSaveLeavesCurrentTargetUnchanged() {
        doThrow(new IllegalStateException("Cannot save")).when(preferences)
                .saveLauncherTargetVelocity(1275);

        org.junit.Assert.assertThrows(IllegalStateException.class, () -> dpad(true, false));

        verify(mechanisms, never()).setLauncherTargetVelocity(anyInt());
    }

    private void dpad(boolean up, boolean down) {
        gamepad.dpad_up = up;
        gamepad.dpad_down = down;
        controller.update(gamepad);
    }

    @Test
    public void leftBumperScalesTranslationAndRotationAndReleaseRestoresNormal() {
        gamepad.left_stick_y = -1;
        gamepad.left_stick_x = 1;
        gamepad.right_stick_x = 1;
        gamepad.left_bumper = true;
        controller.update(gamepad);
        verify(drive).driveRobotRelative(1, 1, 1, 0.35);

        gamepad.left_bumper = false;
        controller.update(gamepad);
        verify(drive).driveRobotRelative(1, 1, 1, 1);
    }

    @Test
    public void precisionAlsoWorksFieldCentricWithoutChangingMechanismInputs() {
        when(preferences.isFieldCentric()).thenReturn(true);
        controller = new TeleopController(drive, mechanisms, preferences, tracking);
        gamepad.left_stick_y = -0.5f;
        gamepad.right_stick_x = 0.25f;
        gamepad.left_bumper = true;
        gamepad.right_bumper = true;
        gamepad.right_trigger = 1;
        controller.update(gamepad);

        verify(drive).driveFieldRelative(0.5, 0, 0.25, 0.35);
        verify(mechanisms).update(1, true);
    }

    @Test
    public void readinessRumblesOncePerTransitionNotEveryLoop() {
        when(mechanisms.isLauncherReady()).thenReturn(false, true, true, false, true);
        for (int i = 0; i < 5; i++) {
            controller.update(gamepad);
        }
        verify(gamepad, times(2)).rumble(200);
        verify(gamepad).stopRumble();
    }

    @Test
    public void targetLockRumblesContinuouslyUntilLostOrReleasedEvenWhenCentered() {
        gamepad.a = true;
        controller.update(gamepad);
        controller.update(gamepad);
        verify(gamepad, never()).rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
        verify(gamepad, never()).stopRumble();

        when(tracking.hasTarget()).thenReturn(true);
        when(tracking.update(true)).thenReturn(0.0);
        controller.update(gamepad);
        controller.update(gamepad);
        verify(gamepad).rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
        verify(gamepad, never()).stopRumble();

        when(tracking.hasTarget()).thenReturn(false);
        controller.update(gamepad);
        verify(gamepad).stopRumble();
        when(tracking.hasTarget()).thenReturn(true);
        controller.update(gamepad);
        verify(gamepad, times(2)).rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
        gamepad.a = false;
        controller.update(gamepad);
        controller.update(gamepad);
        verify(gamepad, times(2)).stopRumble();
    }

    @Test
    public void launcherReadinessChangesDoNotInterruptTargetLock() {
        gamepad.a = true;
        when(tracking.hasTarget()).thenReturn(true);
        when(mechanisms.isLauncherReady()).thenReturn(false, true, false, true);
        for (int i = 0; i < 4; i++) {
            controller.update(gamepad);
        }
        verify(gamepad).rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
        verify(gamepad, never()).rumble(200);
        verify(gamepad, never()).stopRumble();
        when(tracking.hasTarget()).thenReturn(false);
        controller.update(gamepad);
        verify(gamepad).stopRumble();
    }

    @Test
    public void controllerTelemetryReportsLastAppliedControlsWithoutUpdatingHardware() {
        gamepad.left_bumper = true;
        gamepad.left_trigger = 0.25f;
        gamepad.right_trigger = 0.75f;
        controller.update(gamepad);
        clearInvocations(drive, mechanisms, gamepad, preferences);
        RobotTelemetry output = mock(RobotTelemetry.class);

        controller.publishTelemetry(output);

        verify(output).status("Drive mode", "ROBOT-CENTRIC", RobotTelemetry.Level.INFO);
        verify(output).status("Driving", "BUMPER", RobotTelemetry.Level.WARNING);
        verify(output).number("Drive power limit", 35, "%");
        verify(output).number("Left trigger", 0.25, "");
        verify(output).number("Right trigger", 0.75, "");
        verifyNoInteractions(drive, mechanisms, gamepad, preferences);
        verify(output, never()).beginFrame();
        verify(output, never()).endFrame();
    }

    @Test
    public void configuredDriveAndBumperSpeedsApplyToAimingTranslation() {
        ConfigurationSubsystem configuration = mock(ConfigurationSubsystem.class);
        when(configuration.isLocked()).thenReturn(true);
        when(configuration.getDriveSpeedPercent()).thenReturn(45);
        when(configuration.getBumperSpeedPercent()).thenReturn(80);
        controller.configure(configuration, gamepad);
        gamepad.left_stick_y = -1;
        gamepad.right_stick_x = 0.5f;
        controller.update(gamepad);
        verify(drive).driveRobotRelative(1, 0, 0.5, 0.45);
        gamepad.a = true;
        when(tracking.update(true)).thenReturn(0.2);
        controller.update(gamepad);
        verify(drive).driveRobotRelative(0.45, 0, 0.2, 1, true);
        gamepad.a = false;
        gamepad.left_bumper = true;
        controller.update(gamepad);
        verify(drive).driveRobotRelative(1, 0, 0.5, 0.8);
        gamepad.a = true;
        controller.update(gamepad);
        verify(drive).driveRobotRelative(0.8, 0, 0.2, 1, true);
    }

    @Test
    public void holdingATakesOnlyRotationAndReleaseRestoresRightStick() {
        gamepad.left_stick_y = -0.5f;
        gamepad.left_stick_x = 0.25f;
        gamepad.right_stick_x = -1;
        gamepad.a = true;
        when(tracking.update(true)).thenReturn(0.3);

        controller.update(gamepad);
        verify(drive).driveRobotRelative(0.5, 0.25, 0.3, 1, true);
        gamepad.a = false;
        controller.update(gamepad);
        verify(drive).driveRobotRelative(0.5, 0.25, -1, 1);
    }

    @Test
    public void lostTrackingTargetKeepsTranslationButDoesNotUseRightStick() {
        gamepad.left_stick_y = -1;
        gamepad.left_stick_x = -0.5f;
        gamepad.right_stick_x = 1;
        gamepad.a = true;
        when(tracking.update(true)).thenReturn(0.0);

        controller.update(gamepad);

        verify(drive).driveRobotRelative(1, -0.5, 0, 1, true);
    }

    @Test
    public void trackingUsesBumperSpeedForTranslationButNotRotation() {
        when(preferences.isFieldCentric()).thenReturn(true);
        controller = new TeleopController(drive, mechanisms, preferences, tracking);
        gamepad.a = true;
        gamepad.left_bumper = true;
        gamepad.right_bumper = true;
        gamepad.left_stick_y = -0.5f;
        gamepad.left_stick_x = 0.25f;
        when(tracking.update(true)).thenReturn(-0.2);

        controller.update(gamepad);

        verify(drive).driveFieldRelative(0.175, 0.0875, -0.2, 1, true);
        verify(mechanisms).update(0, true);
        RobotTelemetry output = mock(RobotTelemetry.class);
        controller.publishTelemetry(output);
        verify(output).status("Driving", "AIMING (BUMPER)", RobotTelemetry.Level.WARNING);
        verify(output).number("Drive power limit", 100, "%");
        verify(output).number("Translation input scale", 35, "%");

        gamepad.a = false;
        gamepad.right_stick_x = 0.5f;
        controller.update(gamepad);
        verify(drive).driveFieldRelative(0.5, 0.25, 0.5, 0.35);
    }

    @Test
    public void initOnlySelectsAllianceWithoutDrivingOrVisionTracking() {
        clearInvocations(tracking, drive, mechanisms, preferences);
        gamepad.dpad_left = true;
        gamepad.a = true;
        gamepad.right_bumper = true;
        controller.updateInit(gamepad);

        verify(tracking).selectAlliance(true, false);
        verify(tracking).getAlliance();
        verifyNoMoreInteractions(tracking);
        verifyNoInteractions(drive, mechanisms, preferences);
    }

    @Test
    public void controllerLedsMatchAllianceDuringInitAndOnlyUpdateOnChanges() {
        clearInvocations(gamepad);
        controller.updateInit(gamepad);
        verify(gamepad, never()).setLedColor(anyDouble(), anyDouble(), anyDouble(), anyInt());

        when(tracking.getAlliance()).thenReturn(Alliance.RED);
        gamepad.dpad_left = true;
        controller.updateInit(gamepad);
        controller.updateInit(gamepad);
        verify(gamepad).setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);

        when(tracking.getAlliance()).thenReturn(Alliance.BLUE);
        gamepad.dpad_left = false;
        gamepad.dpad_right = true;
        controller.updateInit(gamepad);
        verify(gamepad).setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
    }

    private void buttons(boolean share, boolean options) {
        gamepad.share = share;
        gamepad.options = options;
        controller.update(gamepad);
    }
}
