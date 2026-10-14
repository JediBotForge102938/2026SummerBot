package org.firstinspires.ftc.teamcode.teaching.session3;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "[S3-01] IMU Sensor Check", group = "Teaching S3")
public class S3_01_IMUSensorTeleOp extends OpMode {
    private IMU imu;

    @Override
    public void init() {
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        telemetry.addLine("IMU initialized. Start, then gently rotate the robot by hand.");
    }

    @Override
    public void loop() {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        telemetry.addData("Yaw / heading", "%.1f degrees",
                angles.getYaw(AngleUnit.DEGREES));
        telemetry.addData("Pitch", "%.1f degrees",
                angles.getPitch(AngleUnit.DEGREES));
        telemetry.addData("Roll", "%.1f degrees",
                angles.getRoll(AngleUnit.DEGREES));
        telemetry.update();
    }
}
