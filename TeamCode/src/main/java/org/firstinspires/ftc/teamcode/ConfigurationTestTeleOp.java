package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.ConfigurationSubsystem;
import org.firstinspires.ftc.teamcode.control.DriveModePreferences;
import org.firstinspires.ftc.teamcode.subsystems.RobotProfile;

@TeleOp(name = "Configuration Test", group = "2 - (Tests)")
public class ConfigurationTestTeleOp extends OpMode {
    private final ConfigurationSubsystem configuration = new ConfigurationSubsystem();
    private IMU imu;

    @Override
    public void init() {
        RobotProfile profile = RobotProfile.loadActive(null);
        imu = hardwareMap.get(IMU.class, profile.imuName);
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                profile.imuLogo, profile.imuUsb);
        if (!imu.initialize(new IMU.Parameters(orientation))) {
            throw new IllegalStateException("Unable to initialize the configured drive IMU");
        }
        configuration.init(telemetry, gamepad1, new DriveModePreferences(hardwareMap.appContext));
        configuration.setRobotProfileName(profile.name);
        publish();
    }

    @Override
    public void init_loop() {
        configuration.updateInit(gamepad1);
        publish();
    }

    @Override
    public void start() {
        if (!configuration.isLocked()) {
            imu.resetYaw();
            configuration.lock();
        }
        publish();
    }

    @Override
    public void loop() {
        publish();
    }

    private void publish() {
        telemetry.clearAll();
        configuration.publish();
        double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        if (Double.isFinite(yaw)) {
            if (configuration.isLocked()) {
                // Display [0, 360) so the blue starting reference remains visibly 180 degrees.
                double heading = (yaw + configuration.getStartingHeadingDegrees()) % 360;
                telemetry.addData("Field heading (deg)", "%.2f",
                        heading < 0 ? heading + 360 : heading);
            }
        } else {
            telemetry.addLine("<b><font color='#FF6666'>IMU ERROR: invalid yaw</font></b>");
        }
        telemetry.update();
    }
}
