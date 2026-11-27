package org.firstinspires.ftc.teamcode.pedro;

import static org.junit.Assert.assertEquals;
import static org.mockito.Mockito.*;

import com.pedropathing.revhub.localizers.OTOSLocalizer;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.io.File;
import org.firstinspires.ftc.teamcode.subsystems.RobotProfile;
import org.junit.Test;
import org.mockito.ArgumentCaptor;

public class PedroOtosMountingTest {
    @Test
    public void physicalProfilesApplyMountingRotationToSensorOffset() {
        assertSensorHeading("biobuzz-starterbot", "starterbot", Math.PI / 2);
        assertSensorHeading("biobuzz-competitionbot", "competitionbot", 0);
    }

    @Test
    public void tuningConfigurationUsesSameMountingRotation() {
        HardwareMap hardware = mock(HardwareMap.class);
        SparkFunOTOS sensor = sensor(hardware);
        new OTOSLocalizer(hardware, PedroConstants.otosConfig("otos", -90, 1, 1));
        ArgumentCaptor<SparkFunOTOS.Pose2D> offset = ArgumentCaptor.forClass(SparkFunOTOS.Pose2D.class);
        verify(sensor).setOffset(offset.capture());
        assertEquals(0, normalized(offset.getValue().h), 1e-9);
    }

    private static void assertSensorHeading(String name, String robot, double expectedHeading) {
        RobotProfile profile = RobotProfile.load(new File("../robot-configs"), name, robot);
        HardwareMap hardware = mock(HardwareMap.class);
        SparkFunOTOS sensor = sensor(hardware);
        new OTOSLocalizer(hardware, PedroConstants.otosConfig(profile));
        ArgumentCaptor<SparkFunOTOS.Pose2D> offset = ArgumentCaptor.forClass(SparkFunOTOS.Pose2D.class);
        verify(sensor).setOffset(offset.capture());
        assertEquals(expectedHeading, normalized(offset.getValue().h), 1e-9);
    }

    private static double normalized(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    private static SparkFunOTOS sensor(HardwareMap hardware) {
        SparkFunOTOS sensor = mock(SparkFunOTOS.class);
        when(hardware.get(SparkFunOTOS.class, "otos")).thenReturn(sensor);
        when(sensor.getPosition()).thenReturn(new SparkFunOTOS.Pose2D());
        when(sensor.getVelocity()).thenReturn(new SparkFunOTOS.Pose2D());
        return sensor;
    }
}
