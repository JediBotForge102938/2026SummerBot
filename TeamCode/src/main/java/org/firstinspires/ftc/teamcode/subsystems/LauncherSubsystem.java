package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class LauncherSubsystem {
    private static final int TARGET_VELOCITY = 1250;
    private static final int MIN_VELOCITY = 1200;

    private DcMotorEx launcher;
    private CRServo windmillServo;

    public void init(HardwareMap hardwareMap) {
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmillServo = hardwareMap.get(CRServo.class, "windmillServo");

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(40, 0, 0, 12.5));

        windmillServo.setPower(0);
        windmillServo.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    /**
     * Updates launcher and windmill output and reports whether the windmill is running.
     */
    public boolean update(boolean launchRequested) {
        if (launchRequested) {
            launcher.setVelocity(TARGET_VELOCITY);
        } else {
            launcher.setVelocity(0);
        }

        boolean windmillRunning = launchRequested && launcher.getVelocity() > MIN_VELOCITY;
        windmillServo.setPower(windmillRunning ? 1 : 0);
        return windmillRunning;
    }
}
