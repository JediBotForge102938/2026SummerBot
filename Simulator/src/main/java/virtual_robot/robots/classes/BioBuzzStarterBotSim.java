package virtual_robot.robots.classes;

import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorExImpl;
import com.qualcomm.robotcore.hardware.configuration.MotorType;
import javafx.fxml.FXML;
import javafx.scene.Group;
import javafx.scene.shape.Circle;
import javafx.scene.shape.Rectangle;
import javafx.scene.transform.Rotate;
import local_simulator_config.RobotHardwareConfig;
import org.dyn4j.geometry.Transform;
import virtual_robot.controller.BotConfig;
import virtual_robot.controller.VirtualField;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

@BotConfig(name = "BioBuzz: StarterBot Mecanum", filename = "biobuzz_starterbot")
public class BioBuzzStarterBotSim extends MecanumPhysicsBase {
    private static final String ROBOT_CONFIG_RESOURCE = "/robot-configs/biobuzz-starterbot.xml";
    private static final int STARTING_COLUMN = 3;
    private static final int STARTING_ROW = 6;

    @FXML
    private Rectangle launcherVisual;
    @FXML
    private Rectangle intakeVisual0;
    @FXML
    private Rectangle intakeVisual1;
    @FXML
    private Rectangle intakeVisual2;
    @FXML
    private Rectangle intakeVisual3;
    @FXML
    private Rectangle leftIntakeServoVisual;
    @FXML
    private Rectangle rightIntakeServoVisual;
    @FXML
    private Group windmillVisual;

    private DcMotorExImpl intake;
    private DcMotorExImpl launcher;
    private CRServoImpl leftIntakeServo;
    private CRServoImpl rightIntakeServo;
    private CRServoImpl windmillServo;
    private double launcherAngle;
    private double intakeAngle;

    @Override
    protected void createHardwareMap() {
        super.createHardwareMap();
        hardwareMap.setActive(true);

        List<RobotHardwareConfig.Device> devices =
                RobotHardwareConfig.loadDevices(ROBOT_CONFIG_RESOURCE);
        Set<String> wheelSlots = new HashSet<>();
        for (RobotHardwareConfig.Device device : devices) {
            if (device.type() == RobotHardwareConfig.DeviceType.MOTOR) {
                String wheelSlot = internalMotorName(device.name());
                if (wheelSlot != null) {
                    if (!wheelSlots.add(wheelSlot)) {
                        throw new IllegalArgumentException(
                                "Multiple configured motors map to simulator wheel " + wheelSlot);
                    }
                    hardwareMap.put(device.name(), hardwareMap.get(DcMotorEx.class, wheelSlot));
                } else {
                    DcMotorExImpl mechanismMotor =
                            new DcMotorExImpl(MotorType.Neverest40, motorController1, device.port());
                    hardwareMap.put(device.name(), mechanismMotor);
                    if (device.name().equals("intake")) {
                        intake = mechanismMotor;
                    } else if (device.name().equals("launcher")) {
                        launcher = mechanismMotor;
                    }
                }
            } else {
                CRServoImpl mechanismServo = new CRServoImpl(360);
                hardwareMap.put(device.name(), mechanismServo);
                if (device.name().equals("left_intake_servo")) {
                    leftIntakeServo = mechanismServo;
                } else if (device.name().equals("right_intake_servo")) {
                    rightIntakeServo = mechanismServo;
                } else if (device.name().equals("windmillServo")) {
                    windmillServo = mechanismServo;
                }
            }
        }
        if (wheelSlots.size() != 4) {
            throw new IllegalArgumentException(
                    "BioBuzz StarterBot simulator requires four configured drive motors, found "
                            + wheelSlots.size());
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
    public synchronized void updateStateAndSensors(double millis) {
        super.updateStateAndSensors(millis);
        intake.update(millis);
        launcher.update(millis);
        intakeAngle += intake.getVelocity() * 360.0 / 28.0 * millis / 1000.0;
        launcherAngle += launcher.getVelocity() * 360.0 / 28.0 * millis / 1000.0;
        leftIntakeServo.updatePositionDegrees(millis);
        rightIntakeServo.updatePositionDegrees(millis);
        windmillServo.updatePositionDegrees(millis);
    }

    @Override
    public synchronized void updateDisplay() {
        super.updateDisplay();
        ((Rotate) launcherVisual.getTransforms().get(0)).setAngle(launcherAngle);
        ((Rotate) intakeVisual0.getTransforms().get(0)).setAngle(intakeAngle);
        ((Rotate) intakeVisual1.getTransforms().get(0)).setAngle(intakeAngle);
        ((Rotate) intakeVisual2.getTransforms().get(0)).setAngle(intakeAngle);
        ((Rotate) intakeVisual3.getTransforms().get(0)).setAngle(intakeAngle);
        ((Rotate) leftIntakeServoVisual.getTransforms().get(0))
                .setAngle(leftIntakeServo.getPositionDegrees());
        ((Rotate) rightIntakeServoVisual.getTransforms().get(0))
                .setAngle(rightIntakeServo.getPositionDegrees());
        ((Rotate) windmillVisual.getTransforms().get(0))
                .setAngle(windmillServo.getPositionDegrees());
    }

    @Override
    protected void setUpDisplayGroup(Group group) {
        super.setUpDisplayGroup(group);
        launcherVisual.getTransforms().add(new Rotate(0, 37.5, 80));
        intakeVisual0.getTransforms().add(new Rotate(0, 12.25, 2.5));
        intakeVisual1.getTransforms().add(new Rotate(0, 29.0833, 2.5));
        intakeVisual2.getTransforms().add(new Rotate(0, 45.9167, 2.5));
        intakeVisual3.getTransforms().add(new Rotate(0, 62.75, 2.5));
        leftIntakeServoVisual.getTransforms().add(new Rotate(0, 0, -4));
        rightIntakeServoVisual.getTransforms().add(new Rotate(0, 75, -4));
        windmillVisual.getTransforms().add(new Rotate(0, 37.5, 37.5));
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
        return null;
    }
}
