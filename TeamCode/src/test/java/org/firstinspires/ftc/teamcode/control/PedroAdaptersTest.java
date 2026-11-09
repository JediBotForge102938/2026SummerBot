package org.firstinspires.ftc.teamcode.control;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TrackingSubsystem;
import org.junit.Test;

public class PedroAdaptersTest {
    @Test public void velocityVectorUsesCartesianComponents() {
        TrackingSubsystem tracking = mock(TrackingSubsystem.class);
        when(tracking.getVelocityX()).thenReturn(3.0);
        when(tracking.getVelocityY()).thenReturn(-4.0);
        PedroTrackingLocalizer localizer = new PedroTrackingLocalizer(tracking);
        assertEquals(3, localizer.getVelocityVector().getXComponent(), 1e-9);
        assertEquals(-4, localizer.getVelocityVector().getYComponent(), 1e-9);
        assertEquals(5, localizer.getVelocityVector().getMagnitude(), 1e-9);
    }

    @Test public void outputPreservesSharedPhysicalSignsAndUsesFreeSpeedConstants() {
        DriveSubsystem drive = mock(DriveSubsystem.class);
        PedroDriveOutput output = new PedroDriveOutput(drive);
        output.setXVelocity(25);
        output.setYVelocity(20);
        assertEquals(25, output.xVelocity(), 0);
        assertEquals(20, output.yVelocity(), 0);
        output.setMaxPowerScaling(1);
        output.arcadeDrive(0.3, 0.2, -0.1);
        verify(drive).driveRobotRelative(0.3, -0.2, 0.1, 1, true);
        output.setMaxPowerScaling(0.5);
        output.arcadeDrive(0.3, 0.2, -0.1);
        verify(drive).driveRobotRelative(0.6, -0.4, 0.2, 0.5, true);
        assertThrows(IllegalStateException.class,
                () -> output.arcadeDrive(Double.NaN, 0, 0));
        verify(drive).driveRobotRelative(0, 0, 0);
    }
}
