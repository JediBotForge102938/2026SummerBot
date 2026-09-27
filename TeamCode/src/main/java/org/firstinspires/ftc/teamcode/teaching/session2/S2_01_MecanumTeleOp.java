package org.firstinspires.ftc.teamcode.teaching.session2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "[S2-01] Mecanum", group = "Teaching S2")
public class S2_01_MecanumTeleOp extends OpMode {
    private static final double LESSON_SPEED = 0.5;
    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;

    @Override
    public void init() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        for (DcMotor motor : new DcMotor[] {leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive}) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1.0);
        leftFrontDrive.setPower(LESSON_SPEED * (forward + strafe + rotate) / denominator);
        rightFrontDrive.setPower(LESSON_SPEED * (forward - strafe - rotate) / denominator);
        leftBackDrive.setPower(LESSON_SPEED * (forward - strafe + rotate) / denominator);
        rightBackDrive.setPower(LESSON_SPEED * (forward + strafe - rotate) / denominator);
        telemetry.addData("Mecanum", "forward %.2f strafe %.2f turn %.2f", forward, strafe, rotate);
    }
}
