package virtual_robot.robots.classes;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOSInternal;
import com.qualcomm.robotcore.hardware.DcMotorControllerImpl;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorExImpl;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.configuration.MotorType;
import local_simulator_config.RobotHardwareConfig;
import local_simulator_config.StarterBotOTOS;
import org.firstinspires.ftc.teamcode.subsystems.ActiveRobotProfileSource;
import org.firstinspires.ftc.teamcode.subsystems.RobotProfile;
import virtual_robot.controller.BotConfig;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

@BotConfig(name = "biobuzz-competitionbot", filename = "biobuzz_competitionbot")
public class BioBuzzCompetitionBotSim extends MecanumPhysicsBase {
    private final List<DcMotorExImpl> mechanismMotors = new ArrayList<>();

    @Override
    protected void createHardwareMap() {
        super.createHardwareMap();
        ActiveRobotProfileSource.select("biobuzz-competitionbot");
        hardwareMap.setActive(true);
        RobotProfile profile = RobotProfile.loadActive("competitionbot");
        hardwareMap.put(profile.otosName, new StarterBotOTOS(
                hardwareMap.get(SparkFunOTOSInternal.class, "sensor_otos"), profile));
        hardwareMap.put(profile.limelightName, new Limelight3A());

        Set<String> wheels = new HashSet<>();
        for (RobotHardwareConfig.Device device : RobotHardwareConfig.loadDevices(
                "/robot-configs/biobuzz-competitionbot.xml")) {
            if (device.type() == RobotHardwareConfig.DeviceType.MOTOR) {
                String wheel = RobotHardwareConfig.bioBuzzDriveWheelSlot(device.name());
                if (wheel != null) {
                    if (!wheels.add(wheel)) {
                        throw new IllegalArgumentException("Duplicate simulated wheel " + wheel);
                    }
                    hardwareMap.put(device.name(), hardwareMap.get(DcMotorEx.class, wheel));
                } else {
                    DcMotorExImpl motor = new DcMotorExImpl(
                            MotorType.Neverest40, new DcMotorControllerImpl(), device.port());
                    mechanismMotors.add(motor);
                    hardwareMap.put(device.name(), motor);
                }
            } else if (device.type() == RobotHardwareConfig.DeviceType.SERVO) {
                hardwareMap.put(device.name(), new ServoImpl());
            } else {
                throw new IllegalArgumentException("Unsupported CompetitionBot servo: " + device.name());
            }
        }
        if (wheels.size() != 4) {
            throw new IllegalArgumentException("CompetitionBot requires four drive motors; found " + wheels.size());
        }
        hardwareMap.setActive(false);
    }

    @Override
    public synchronized void updateStateAndSensors(double millis) {
        super.updateStateAndSensors(millis);
        for (DcMotorExImpl motor : mechanismMotors) motor.update(millis);
    }
}
