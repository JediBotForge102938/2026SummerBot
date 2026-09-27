package virtual_robot.robots.classes;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import local_simulator_config.RobotHardwareConfig;
import virtual_robot.controller.BotConfig;
import virtual_robot.controller.VirtualField;

import java.util.List;
import java.util.HashSet;
import java.util.Set;
import org.dyn4j.geometry.Transform;

@BotConfig(name = "Stryker: StarterBot", filename = "starterbot")
public class StarterBotMecanumSim extends MecanumPhysicsBase {
    private static final String ROBOT_CONFIG_RESOURCE = "/robot-configs/starterbot-drive-only.xml";
    private static final int STARTING_COLUMN = 3;
    private static final int STARTING_ROW = 6;

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

        Set<String> wheelSlots = new HashSet<>();
        for (RobotHardwareConfig.Motor motor : configuredMotors) {
            String wheelSlot = internalMotorName(motor.name());
            if (!wheelSlots.add(wheelSlot)) {
                throw new IllegalArgumentException(
                        "Multiple configured motors map to simulator wheel " + wheelSlot);
            }
            hardwareMap.put(motor.name(), hardwareMap.get(DcMotorEx.class, wheelSlot));
        }
        hardwareMap.setActive(false);
    }

    @Override
    public void initialize() {
        super.initialize();

        double cellSize = VirtualField.FIELD_WIDTH / 6.0;
        x = (STARTING_COLUMN - 0.5) * cellSize - VirtualField.HALF_FIELD_WIDTH;
        y = VirtualField.HALF_FIELD_WIDTH - (STARTING_ROW - 0.5) * cellSize;

        Transform transform = new Transform();
        transform.translate(
                x / VirtualField.PIXELS_PER_METER,
                y / VirtualField.PIXELS_PER_METER);
        chassisBody.setTransform(transform);
        chassisBody.setLinearVelocity(0, 0);
        chassisBody.setAngularVelocity(0);
    }

    @Override
    protected void setUpDisplayGroup(javafx.scene.Group group) {
        super.setUpDisplayGroup(group);
        updateDisplay();
    }

    private static String internalMotorName(String configuredName) {
        String name = configuredName.toLowerCase().replace('-', '_').replace(' ', '_');
        if (name.contains("left_front") || name.contains("front_left")) {
            return "front_left_motor";
        }
        if (name.contains("right_front") || name.contains("front_right")) {
            return "front_right_motor";
        }
        if (name.contains("left_back") || name.contains("back_left")) {
            return "back_left_motor";
        }
        if (name.contains("right_back") || name.contains("back_right")) {
            return "back_right_motor";
        }
        throw new IllegalArgumentException(
                "Configured motor name does not identify a StarterBot wheel position: " + configuredName);
    }
}
