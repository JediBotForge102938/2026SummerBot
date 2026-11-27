package org.firstinspires.ftc.teamcode.pedro.procedures;

import static org.junit.Assert.*;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;

import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.localization.Localizer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.junit.Test;

public class VelocityTunerTest {
    @Test
    public void forwardStopAlwaysStopsDrivetrain() throws Exception {
        Localizer localizer = mock(Localizer.class, RETURNS_DEEP_STUBS);
        Drivetrain drivetrain = mock(Drivetrain.class);
        ForwardVelocity tuner = spy(new ForwardVelocity(h -> localizer, h -> drivetrain, 48));
        tuner.hardwareMap = mock(HardwareMap.class);
        doNothing().when(tuner).waitForStart();
        doReturn(true, false).when(tuner).opModeIsActive();

        assertThrows(InterruptedException.class, tuner::runTuningOpMode);
        verify(drivetrain).drive(any(), eq(true));
        verify(drivetrain).stop();
    }

    @Test
    public void strafeInvalidPositionStopsDrivetrain() throws Exception {
        Localizer localizer = mock(Localizer.class, RETURNS_DEEP_STUBS);
        when(localizer.pose().y()).thenReturn(Double.NaN);
        Drivetrain drivetrain = mock(Drivetrain.class);
        StrafeVelocity tuner = spy(new StrafeVelocity(h -> localizer, h -> drivetrain, 48));
        tuner.hardwareMap = mock(HardwareMap.class);
        doNothing().when(tuner).waitForStart();
        doReturn(true).when(tuner).opModeIsActive();

        assertThrows(IllegalStateException.class, tuner::runTuningOpMode);
        verify(drivetrain).stop();
        verify(drivetrain, never()).drive(any(), anyBoolean());
    }
}
