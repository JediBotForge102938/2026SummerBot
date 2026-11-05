package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** StarterBot feeder; the competition positional gate is configured separately. */
public class WindmillSubsystem implements LauncherFeeder {
    private CRServo servo;

    public void init(HardwareMap hardwareMap) {
        servo = hardwareMap.get(CRServo.class, "windmillServo");
        servo.setDirection(DcMotorSimple.Direction.FORWARD);
        servo.setPower(0);
    }

    @Override
    public void update(boolean feed) {
        servo.setPower(feed ? 1 : 0);
    }
}
