package virtual_robot.robots.classes;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import local_simulator_config.RobotHardwareConfig;
import virtual_robot.controller.BotConfig;

import java.util.List;

@BotConfig(name = "Stryker: StarterBot", filename = "starterbot")
public class StarterBotMecanumSim extends MecanumPhysicsBase {
    private static final String ROBOT_CONFIG_RESOURCE = "/robot-configs/starterbot-drive-only.xml";

    @Override
    protected boolean isWheelMechanicallyReversed(int wheelIndex) {
        return wheelIndex == 0 || wheelIndex == 3;
    }

    @Override
    protected void createHardwareMap() {
        super.createHardwareMap();

        hardwareMap.setActive(true);
        List<RobotHardwareConfig.Motor> configuredMotors =
                RobotHardwareConfig.loadMotors(ROBOT_CONFIG_RESOURCE);
        if (configuredMotors.size() != 4) {
            throw new IllegalArgumentException(
                    "StarterBot simulator requires exactly four configured drive motors, found "
                            + configuredMotors.size());
        }

        for (RobotHardwareConfig.Motor motor : configuredMotors) {
            hardwareMap.put(motor.name(), hardwareMap.get(DcMotorEx.class, internalMotorName(motor.port())));
        }
        hardwareMap.setActive(false);
    }

    private static String internalMotorName(int port) {
        return switch (port) {
            case 0 -> "back_left_motor";
            case 1 -> "front_left_motor";
            case 2 -> "front_right_motor";
            case 3 -> "back_right_motor";
            default -> throw new IllegalArgumentException(
                    "StarterBot drive motor port must be between 0 and 3, found " + port);
        };
    }
}
