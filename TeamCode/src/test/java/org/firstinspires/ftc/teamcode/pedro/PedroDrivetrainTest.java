package org.firstinspires.ftc.teamcode.pedro;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.io.File;
import org.firstinspires.ftc.teamcode.subsystems.RobotProfile;
import org.junit.Test;
import org.mockito.MockedStatic;

public class PedroDrivetrainTest {
    @Test
    public void physicalStarterReversesOnlyStrafeThroughTheDriveFactory() {
        Mecanum drivetrain = drivetrain("starterbot", false);
        drivetrain.drive(new DrivePowers(0.2, 0.3, 0.1), false);
        assertArrayEquals(new double[] {0.4, 0, -0.2, 0.6}, drivetrain.wheelPowers, 1e-9);
        assertArrayEquals(new double[] {0.3, 0.3, 0.3, 0.3},
                drivetrain.computeWheelPowersUnnormalized(new DrivePowers(0.3, 0, 0)), 1e-9);
        assertArrayEquals(new double[] {-0.3, 0.3, -0.3, 0.3},
                drivetrain.computeWheelPowersUnnormalized(new DrivePowers(0, 0, 0.3)), 1e-9);
    }

    @Test
    public void competitionKeepsStandardStrafeMix() {
        Mecanum drivetrain = drivetrain("competitionbot", false);
        drivetrain.drive(new DrivePowers(0.2, 0.3, 0.1), false);
        assertArrayEquals(new double[] {-0.2, 0.6, 0.4, 0}, drivetrain.wheelPowers, 1e-9);
    }

    @Test
    public void simulatorStarterKeepsItsExistingWheelMappingWithoutAnotherStrafeFlip() {
        Mecanum drivetrain = drivetrain("starterbot", true);
        drivetrain.drive(new DrivePowers(0, 0.3, 0), false);
        assertArrayEquals(new double[] {-0.3, 0.3, 0.3, -0.3}, drivetrain.wheelPowers, 1e-9);
        assertEquals("left_back_drive", drivetrain.config.frontLeftName.get());
        assertEquals("right_back_drive", drivetrain.config.frontRightName.get());
    }

    private static Mecanum drivetrain(String robot, boolean simulator) {
        RobotProfile profile = spy(RobotProfile.load(new File("../robot-configs"),
                "biobuzz-" + robot, robot));
        doReturn(simulator).when(profile).isSimulatorRuntime();
        HardwareMap hardware = mock(HardwareMap.class);
        for (String name : new String[] {"left_front_drive", "right_front_drive",
                "left_back_drive", "right_back_drive"}) {
            when(hardware.get(DcMotorEx.class, name)).thenReturn(mock(DcMotorEx.class));
        }
        try (MockedStatic<RobotProfile> profiles = mockStatic(RobotProfile.class)) {
            profiles.when(() -> RobotProfile.loadActive(null)).thenReturn(profile);
            return PedroConstants.drivetrain(hardware);
        }
    }
}
