package org.firstinspires.ftc.teamcode.teaching.session2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "[S2-02] Speed toggle - Exercise", group = "Teaching S2")
public class S2_02_SpeedToggleExerciseTeleOp extends OpMode {
    private static final double LESSON_SPEED = 0.5;
    private DcMotor leftFront, rightFront, leftBack, rightBack;
    private boolean slow;
    private boolean wasPressed;

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFront = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBack = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBack = hardwareMap.get(DcMotor.class, "right_back_drive");
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        for (DcMotor motor : new DcMotor[] {leftFront, rightFront, leftBack, rightBack}) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    @Override
    public void loop() {
        boolean pressed = gamepad1.right_bumper;
        if (pressed) {
            if (slow == false) {
                slow = true;
            } else {
                slow = false;
            }
        }
        wasPressed = pressed;
        double forward = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        double scale = LESSON_SPEED * (slow ? 0.35 : 1.0);
        leftFront.setPower(scale * (forward + turn));
        leftBack.setPower(scale * (forward + turn));
        rightFront.setPower(scale * (forward - turn));
        rightBack.setPower(scale * (forward - turn));
        telemetry.addData("Mode", slow ? "SLOW" : "FULL");
    }
}
