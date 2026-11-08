package local_simulator_config;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import javafx.fxml.FXMLLoader;
import javafx.application.Platform;
import javafx.scene.layout.Pane;
import org.dyn4j.dynamics.Body;
import org.dyn4j.world.World;
import org.firstinspires.ftc.teamcode.subsystems.RobotProfile;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FieldMap;
import org.firstinspires.ftc.teamcode.subsystems.TrackingSubsystem;
import org.firstinspires.ftc.teamcode.CompetitionBotTeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.Test;
import org.junit.Assume;
import org.reflections.Reflections;
import virtual_robot.controller.BotConfig;
import virtual_robot.controller.VirtualBot;
import virtual_robot.controller.VirtualRobotController;
import virtual_robot.robots.classes.BioBuzzCompetitionBotSim;
import virtual_robot.robots.classes.BioBuzzStarterBotSim;
import virtual_robot.robots.classes.StarterBotConfigurationExerciseSim;
import virtual_robot.robots.classes.StarterBotMecanumSim;

public class SimulatorConfigurationsTest {
    @Test public void configurationNamesAndResourcesMatchXmlFiles() {
        assertConfig(BioBuzzStarterBotSim.class, "biobuzz-starterbot");
        assertConfig(BioBuzzCompetitionBotSim.class, "biobuzz-competitionbot");
        assertConfig(StarterBotMecanumSim.class, "starterbot-drive-only");
        assertConfig(StarterBotConfigurationExerciseSim.class, "s2_04_configuration_exercise");
        assertTrue(RobotHardwareConfig.loadDevices("/robot-configs/biobuzz-competitionbot.xml")
                .stream().anyMatch(device -> device.name().equals("gate")
                        && device.type() == RobotHardwareConfig.DeviceType.SERVO));
        java.util.Set<Class<?>> discovered = new Reflections("virtual_robot.robots.classes")
                .getTypesAnnotatedWith(BotConfig.class);
        assertTrue(discovered.contains(BioBuzzStarterBotSim.class));
        assertTrue(discovered.contains(BioBuzzCompetitionBotSim.class));
        assertTrue(discovered.contains(StarterBotMecanumSim.class));
        assertTrue(discovered.contains(StarterBotConfigurationExerciseSim.class));
        assertEquals("front_left_motor", RobotHardwareConfig.driveWheelSlot("left_front_drive"));
        assertEquals("back_left_motor", RobotHardwareConfig.bioBuzzDriveWheelSlot("left_front_drive"));
        assertEquals("front_left_motor", RobotHardwareConfig.bioBuzzDriveWheelSlot("left_back_drive"));
        assertEquals("back_right_motor", RobotHardwareConfig.bioBuzzDriveWheelSlot("right_front_drive"));
        assertEquals("front_right_motor", RobotHardwareConfig.bioBuzzDriveWheelSlot("right_back_drive"));
    }

    private void assertConfig(Class<?> robot, String name) {
        BotConfig config = robot.getAnnotation(BotConfig.class);
        assertNotNull(config);
        assertFalse(config.disabled());
        assertEquals(name, config.name());
        assertNotNull(robot.getResource("/virtual_robot/robots/fxml/" + config.filename() + ".fxml"));
    }

    @Test public void competitionModelLoadsItsProfileAndHardware() throws Exception {
        Assume.assumeTrue("A graphical display is required",
                System.getenv("DISPLAY") != null || System.getProperty("os.name").startsWith("Windows")
                        || System.getProperty("os.name").startsWith("Mac"));
        Platform.startup(() -> { });
        try {
            VirtualRobotController controller = mock(VirtualRobotController.class);
            when(controller.getFieldPane()).thenReturn(new Pane());
            when(controller.getWorld()).thenReturn(new World<Body>());
            java.lang.reflect.Method setController = VirtualBot.class.getDeclaredMethod(
                    "setController", VirtualRobotController.class);
            setController.setAccessible(true);
            setController.invoke(null, controller);
            FXMLLoader loader = new FXMLLoader(getClass().getResource(
                    "/virtual_robot/robots/fxml/biobuzz_competitionbot.fxml"));
            loader.load();
            VirtualBot robot = loader.getController();
            HardwareMap hardware = robot.getHardwareMap();
            hardware.setActive(true);
            assertEquals("biobuzz-competitionbot", RobotProfile.loadActive("competitionbot").name);
            for (String name : new String[] {"left_front_drive", "right_front_drive",
                    "left_back_drive", "right_back_drive", "left_launcher", "right_launcher", "intake"}) {
                assertNotNull(hardware.get(DcMotorEx.class, name));
            }
            assertNotNull(hardware.get(Servo.class, "gate"));
            assertNotNull(hardware.get(SparkFunOTOS.class, "otos"));
            assertFalse(hardware.get(Limelight3A.class, "limelight").getLatestResult().isValid());
            assertThrows(IllegalStateException.class, () -> RobotProfile.loadActive("starterbot"));

            java.lang.reflect.Field mapField = VirtualRobotController.class.getDeclaredField("hardwareMap");
            mapField.setAccessible(true);
            mapField.set(controller, hardware);
            for (String fieldName : new String[] {"gamePad1", "gamePad2"}) {
                java.lang.reflect.Field field = VirtualRobotController.class.getDeclaredField(fieldName);
                field.setAccessible(true);
                field.set(controller, new Gamepad());
            }
            OpMode.setVirtualRobotController(controller);
            CompetitionBotTeleOp op = new CompetitionBotTeleOp();
            op.telemetry = mock(Telemetry.class);
            op.init();
            op.init_loop();
            op.start();
            op.loop();
            op.stop();

            FXMLLoader basic = new FXMLLoader(getClass().getResource(
                    "/virtual_robot/robots/fxml/starterbot.fxml"));
            basic.load();
            assertThrows(IllegalStateException.class, () -> RobotProfile.loadActive(null));
            FXMLLoader starter = new FXMLLoader(getClass().getResource(
                    "/virtual_robot/robots/fxml/biobuzz_starterbot.fxml"));
            starter.load();
            assertEquals("biobuzz-starterbot", RobotProfile.loadActive("starterbot").name);

            for (String model : new String[] {"biobuzz_starterbot", "biobuzz_competitionbot"}) {
                assertMotion(model, 1, 0, 0, false, 1);
                assertMotion(model, -1, 0, 0, false, 1);
                assertMotion(model, 0, 1, 0, false, 0);
                assertMotion(model, 0, -1, 0, false, 0);
                assertMotion(model, 0, 1, 0, true, 0);
                assertMotion(model, 0, -1, 0, true, 0);
                assertMotion(model, 0, 0, 1, false, 2);
                assertMotion(model, 0, 0, -1, false, 2);
            }
        } finally {
            Platform.exit();
        }
    }

    private void assertMotion(String model, double forward, double strafe, double rotate,
            boolean fieldRelative, int axis) throws Exception {
        VirtualRobotController controller = new VirtualRobotController();
        java.lang.reflect.Field pane = VirtualRobotController.class.getDeclaredField("fieldPane");
        pane.setAccessible(true);
        pane.set(controller, new Pane());
        java.lang.reflect.Field halfWidth = VirtualRobotController.class.getDeclaredField("halfFieldWidth");
        halfWidth.setAccessible(true);
        halfWidth.setDouble(controller, 324);
        World<Body> world = controller.getWorld();
        java.lang.reflect.Method setController = VirtualBot.class.getDeclaredMethod(
                "setController", VirtualRobotController.class);
        setController.setAccessible(true);
        setController.invoke(null, controller);
        FXMLLoader loader = new FXMLLoader(getClass().getResource(
                "/virtual_robot/robots/fxml/" + model + ".fxml"));
        loader.load();
        VirtualBot robot = loader.getController();
        java.lang.reflect.Field bodyField = VirtualBot.class.getDeclaredField("chassisBody");
        bodyField.setAccessible(true);
        Body body = (Body) bodyField.get(robot);
        body.setTransform(new org.dyn4j.geometry.Transform());
        robot.updateStateAndSensors(20);
        HardwareMap hardware = robot.getHardwareMap();
        hardware.setActive(true);
        DriveSubsystem drive = new DriveSubsystem();
        drive.init(hardware);
        drive.resetHeading();
        TrackingSubsystem tracking = new TrackingSubsystem();
        tracking.init(hardware, RobotProfile.loadActive(null));
        tracking.start(new FieldMap.Pose(0, 0, 0));
        if (fieldRelative) drive.driveFieldRelative(forward, strafe, rotate);
        else drive.driveRobotRelative(forward, strafe, rotate);
        double x = robot.getX(), y = robot.getY(), heading = robot.getHeadingRadians();
        for (int i = 0; i < 10; i++) {
            world.updatev(0.02);
            robot.updateStateAndSensors(20);
        }
        double dx = robot.getX() - x;
        double dy = robot.getY() - y;
        double dh = robot.getHeadingRadians() - heading;
        tracking.update(drive.getHeadingDegrees());
        assertNotNull(tracking.getPose());
        if (axis == 0) {
            assertTrue(model + " strafe " + strafe + " dx=" + dx, dx * strafe > 1);
            assertTrue(model + " tracked strafe", tracking.getPose().y * strafe < -0.1);
            assertTrue(model + " strafe dx=" + dx + " dy=" + dy, Math.abs(dy) < Math.abs(dx) * 0.25);
            assertEquals(0, dh, 0.001);
        } else if (axis == 1) {
            assertTrue(model + " forward " + forward + " dy=" + dy, dy * forward > 1);
            assertTrue(model + " tracked forward", tracking.getPose().x * forward > 0.1);
            assertTrue(model + " forward dx=" + dx + " dy=" + dy, Math.abs(dx) < Math.abs(dy) * 0.25);
            assertEquals(0, dh, 0.001);
        } else {
            assertTrue(model + " rotation " + rotate + " dh=" + dh, dh * rotate < -0.01);
            assertEquals(0, dx, 10);
            assertEquals(0, dy, 10);
        }
    }
}
