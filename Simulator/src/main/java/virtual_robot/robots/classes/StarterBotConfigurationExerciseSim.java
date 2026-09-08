package virtual_robot.robots.classes;

import local_simulator_config.RobotHardwareConfig;
import virtual_robot.controller.BotConfig;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

import com.qualcomm.robotcore.hardware.DcMotorEx;

@BotConfig(name = "Stryker: S2-04 Configuration Exercise", filename = "s2_04_configuration_exercise")
public class StarterBotConfigurationExerciseSim extends MecanumPhysicsBase {
    private static final String ROBOT_CONFIG_RESOURCE =
            "/robot-configs/lessons/s2_04_configuration_exercise.xml";

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
                    "Configuration exercise simulator requires exactly four configured drive motors, found "
                            + configuredMotors.size());
        }

        Set<String> wheelSlots = new HashSet<>();
        for (RobotHardwareConfig.Motor motor : configuredMotors) {
            String wheelSlot = internalMotorName(motor.port());
            if (!wheelSlots.add(wheelSlot)) {
                throw new IllegalArgumentException(
                        "Multiple configured motors map to simulator wheel " + wheelSlot);
            }
            hardwareMap.put(motor.name(), hardwareMap.get(DcMotorEx.class, wheelSlot));
        }
        hardwareMap.setActive(false);
    }

    private static String internalMotorName(int port) {
        return switch (port) {
            case 0 -> "back_left_motor";
            case 1 -> "back_right_motor";
            case 2 -> "front_right_motor";
            case 3 -> "front_left_motor";
            default -> throw new IllegalArgumentException(
                    "StarterBot motor port must be between 0 and 3, found " + port);
        };
    }
}
