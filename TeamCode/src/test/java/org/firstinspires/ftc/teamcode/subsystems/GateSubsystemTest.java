package org.firstinspires.ftc.teamcode.subsystems;

import static org.mockito.Mockito.*;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import java.io.File;
import org.junit.Test;
import org.mockito.InOrder;

public class GateSubsystemTest {
    @Test public void initializesClosedAndTracksFeedRequest() {
        RobotProfile profile = RobotProfile.load(new File("../robot-configs"),
                "biobuzz-competitionbot", "competitionbot");
        HardwareMap hardware = mock(HardwareMap.class);
        Servo servo = mock(Servo.class);
        when(hardware.get(Servo.class, "gate")).thenReturn(servo);
        GateSubsystem gate = new GateSubsystem(profile.launcherGate);
        gate.init(hardware);
        gate.update(true);
        gate.update(false);
        InOrder order = inOrder(servo);
        order.verify(servo).setPosition(profile.launcherGate.closedPosition);
        order.verify(servo).setPosition(profile.launcherGate.openPosition);
        order.verify(servo).setPosition(profile.launcherGate.closedPosition);
    }
}
