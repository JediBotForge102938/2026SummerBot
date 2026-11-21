package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.Objects;

public class GateSubsystem implements LauncherFeeder {
    private final RobotProfile.GateConfig config;
    private Servo servo;

    public GateSubsystem(RobotProfile.GateConfig config) {
        this.config = Objects.requireNonNull(config, "Gate configuration");
    }

    public void init(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, config.name);
        update(false);
    }

    @Override
    public void update(boolean feed) {
        servo.setPosition(feed ? config.openPosition : config.closedPosition);
    }
}
