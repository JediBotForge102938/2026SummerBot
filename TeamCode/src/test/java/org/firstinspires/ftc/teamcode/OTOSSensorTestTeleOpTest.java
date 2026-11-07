package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.mockito.ArgumentCaptor;
import org.mockito.MockedConstruction;
import org.mockito.MockedStatic;
import org.firstinspires.ftc.teamcode.subsystems.RobotProfile;

public class OTOSSensorTestTeleOpTest {
    private OTOSSensorTestTeleOp opMode;
    private SparkFunOTOS otos;
    private final SparkFunOTOS.Pose2D pose = new SparkFunOTOS.Pose2D();
    private final SparkFunOTOS.Status status = new SparkFunOTOS.Status();
    private final DcMotor[] motors = new DcMotor[4];
    private MockedConstruction<RevHubOrientationOnRobot> orientation;
    private MockedStatic<RobotProfile> profiles;
    private double time;

    @Before
    public void setUp() {
        RobotProfile profile = TestProfiles.starter();
        profiles = mockStatic(RobotProfile.class);
        profiles.when(() -> RobotProfile.loadActive(null)).thenReturn(profile);
        orientation = mockConstruction(RevHubOrientationOnRobot.class);
        opMode = spy(new OTOSSensorTestTeleOp());
        doAnswer(invocation -> time).when(opMode).getRuntime();
        opMode.hardwareMap = mock(HardwareMap.class);
        opMode.gamepad1 = mock(Gamepad.class);
        opMode.telemetry = mock(Telemetry.class);
        String[] names = {"left_front_drive", "right_front_drive",
                "left_back_drive", "right_back_drive"};
        for (int i = 0; i < motors.length; i++) {
            motors[i] = mock(DcMotor.class);
            when(opMode.hardwareMap.get(DcMotor.class, names[i])).thenReturn(motors[i]);
        }
        IMU imu = mock(IMU.class);
        when(opMode.hardwareMap.get(IMU.class, "imu")).thenReturn(imu);
        when(imu.initialize(any())).thenReturn(true);
        otos = mock(SparkFunOTOS.class);
        when(opMode.hardwareMap.get(SparkFunOTOS.class, "otos")).thenReturn(otos);
        when(otos.calibrateImu()).thenReturn(true);
        when(otos.getStatus()).thenReturn(status);
        doAnswer(invocation -> {
            SparkFunOTOS.Pose2D position = invocation.getArgument(0);
            position.x = pose.x;
            position.y = pose.y;
            position.h = pose.h;
            return null;
        }).when(otos).getPosVelAcc(any(), any(), any());
        doAnswer(invocation -> {
            pose.x = 0;
            pose.y = 0;
            pose.h = 0;
            return null;
        }).when(otos).resetTracking();
        opMode.init();
    }

    @After
    public void tearDown() {
        orientation.close();
        profiles.close();
    }

    @Test
    public void optionsResetsHomeAnywhereOncePerPressAndCancelsReturn() {
        pose.x = -100;
        pose.y = 50;
        pose.h = 90;
        opMode.gamepad1.a = true;
        opMode.loop();
        verify(opMode.telemetry).addData("Drive status", "Returning to home");
        opMode.gamepad1.options = true;
        opMode.loop();
        opMode.loop();
        verify(otos, times(2)).resetTracking();
        clearInvocations((Object[]) motors);
        opMode.gamepad1.options = false;
        opMode.loop();
        assertStopped();
        opMode.gamepad1.options = true;
        opMode.loop();
        verify(otos, times(3)).resetTracking();
    }

    @Test
    public void yStopsAndOverridesOptionsAAndSticksWithoutChangingHome() {
        pose.x = -12;
        opMode.gamepad1.a = true;
        opMode.loop();
        opMode.gamepad1.y = true;
        opMode.gamepad1.options = true;
        opMode.gamepad1.left_stick_y = -1;
        clearInvocations((Object[]) motors);
        opMode.loop();
        assertStopped();
        verify(otos).resetTracking();
        verify(opMode.telemetry).addData("Drive status", "Stopped by Y");
        opMode.gamepad1.y = false;
        opMode.gamepad1.left_stick_y = 0;
        opMode.loop();
        verify(otos).resetTracking();
        opMode.gamepad1.options = false;
        opMode.loop();
        opMode.gamepad1.start = true;
        opMode.loop();
        opMode.loop();
        verify(otos, times(2)).resetTracking();
    }

    @Test
    public void otherControlsDoNotResetHome() {
        pose.x = -12;
        opMode.gamepad1.a = true;
        opMode.gamepad1.dpad_up = true;
        opMode.gamepad1.left_bumper = true;
        opMode.gamepad1.right_trigger = 1;
        opMode.loop();
        opMode.gamepad1.b = true;
        opMode.loop();
        opMode.gamepad1.x = true;
        opMode.loop();
        verify(otos).resetTracking();
    }

    @Test
    public void manualDriveAndReturnOverrideDoNotPulseZeroPower() {
        pose.x = -12;
        opMode.gamepad1.a = true;
        opMode.loop();
        opMode.gamepad1.left_stick_y = -1;
        for (int step = 0; step < 3; step++) {
            clearInvocations((Object[]) motors);
            opMode.loop();
            for (DcMotor motor : motors) {
                verify(motor).setPower(0.45);
                verifyNoMoreInteractions(motor);
            }
        }
    }

    @Test
    public void directReturnUsesRawAxesWithoutFieldRestrictions() {
        pose.x = -100;
        opMode.gamepad1.a = true;
        opMode.loop();
        for (DcMotor motor : motors) {
            assertTrue(latestPower(motor) < 0);
        }
        pose.x = 0;
        pose.y = 12;
        opMode.gamepad1.a = false;
        opMode.loop();
        assertTrue(latestPower(motors[0]) > 0);
        assertTrue(latestPower(motors[1]) < 0);
        assertTrue(latestPower(motors[2]) < 0);
        assertTrue(latestPower(motors[3]) > 0);
        pose.y = 0;
        pose.h = -90;
        opMode.loop();
        assertTrue(latestPower(motors[0]) < 0);
        assertTrue(latestPower(motors[1]) > 0);
    }

    @Test
    public void speedChangesOncePerPressAndWorksBeforeStartWithoutMoving() {
        opMode.gamepad1.dpad_up = true;
        opMode.gamepad1.left_stick_y = -1;
        clearInvocations((Object[]) motors);
        opMode.init_loop();
        opMode.init_loop();
        assertStopped();
        clearInvocations((Object[]) motors);
        opMode.loop();
        for (DcMotor motor : motors) {
            verify(motor).setPower(0.5);
        }
        opMode.gamepad1.dpad_up = false;
        opMode.gamepad1.dpad_down = true;
        opMode.loop();
        for (DcMotor motor : motors) {
            assertEquals(0.45, latestPower(motor), 1e-9);
        }
    }

    @Test
    public void returnStopsAfterSettlingAndDoesNotRestartWhileAHeld() {
        pose.x = -12;
        opMode.gamepad1.a = true;
        opMode.loop();
        pose.x = 0.5;
        opMode.loop();
        time = 0.21;
        opMode.loop();
        verify(opMode.telemetry).addData("Drive status",
                "Home reached (within 1 inch / 3 degrees)");
        pose.x = -12;
        clearInvocations((Object[]) motors);
        opMode.loop();
        assertStopped();
    }

    @Test
    public void warningsTimeoutAndStopCancelReturn() {
        pose.x = -12;
        opMode.gamepad1.a = true;
        opMode.loop();
        status.warnOpticalTracking = true;
        opMode.loop();
        status.warnOpticalTracking = false;
        clearInvocations((Object[]) motors);
        opMode.loop();
        assertStopped();
        opMode.gamepad1.a = false;
        opMode.loop();
        opMode.gamepad1.a = true;
        opMode.loop();
        time = 12;
        opMode.loop();
        verify(opMode.telemetry).addData("Drive status",
                "Return timed out: check tracking and drive directions");
        clearInvocations((Object[]) motors);
        opMode.stop();
        assertStopped();
    }

    @Test
    public void invalidReadingsStopDrive() {
        pose.x = Double.NaN;
        opMode.gamepad1.left_stick_y = -1;
        clearInvocations((Object[]) motors);
        opMode.loop();
        assertStopped();
        verify(opMode.telemetry).addData("Drive status",
                "Drive stopped: OTOS error or invalid readings");
    }

    private double latestPower(DcMotor motor) {
        ArgumentCaptor<Double> power = ArgumentCaptor.forClass(Double.class);
        verify(motor, atLeastOnce()).setPower(power.capture());
        return power.getValue();
    }

    private void assertStopped() {
        for (DcMotor motor : motors) {
            assertEquals(0, latestPower(motor), 0);
        }
    }
}
