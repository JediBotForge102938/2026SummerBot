package local_simulator_config;

import static org.junit.Assert.*;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;

import com.qualcomm.hardware.CommonOdometry;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.hardware.sparkfun.SparkFunOTOSInternal;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.FieldTrackingTestTeleOp;
import org.firstinspires.ftc.teamcode.LandmarkTravelTestTeleOp;
import org.firstinspires.ftc.teamcode.control.LandmarkFieldMap;
import org.firstinspires.ftc.teamcode.control.LandmarkNavigationService;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FieldMap;
import org.firstinspires.ftc.teamcode.subsystems.RobotProfile;
import org.firstinspires.ftc.teamcode.subsystems.TrackingSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ActiveRobotProfileSource;
import org.junit.Before;
import org.junit.Assume;
import org.junit.Test;
import virtual_robot.controller.VirtualRobotController;
import virtual_robot.controller.VirtualBot;
import virtual_robot.controller.VirtualField;
import javafx.application.Platform;
import javafx.fxml.FXMLLoader;
import javafx.scene.layout.Pane;
import org.dyn4j.dynamics.Body;
import org.dyn4j.world.World;

public class SimulatorTrackingTest {
    @Before public void selectStarterProfile() {
        ActiveRobotProfileSource.select("biobuzz-starterbot");
    }
    private static Pose2D pose(double x, double y, double h) {
        return new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, h);
    }

    @Test public void profileAndEmptyLimelightAreAvailable() {
        RobotProfile profile = RobotProfile.loadActive("starterbot");
        assertEquals(9.5, profile.limelightFromBackCm, 0);
        assertThrows(IllegalStateException.class, () -> RobotProfile.loadActive("competitionbot"));
        ActiveRobotProfileSource.select("biobuzz-competitionbot");
        assertEquals("biobuzz-competitionbot", RobotProfile.loadActive("competitionbot").name);
        assertThrows(IllegalStateException.class, () -> RobotProfile.loadActive("starterbot"));
        ActiveRobotProfileSource.select(null);
        assertThrows(IllegalStateException.class, () -> RobotProfile.loadActive(null));
        Limelight3A camera = new Limelight3A();
        camera.start();
        assertFalse(camera.getLatestResult().isValid());
        assertTrue(camera.getLatestResult().getFiducialResults().isEmpty());
        camera.stop();
    }

    @Test public void trackingFollowsTranslationAndRotationOfSimulatedCenter() {
        CommonOdometry odo = CommonOdometry.getInstance();
        Pose2D zero = pose(0, 0, 0);
        odo.update(zero, zero, zero);
        SparkFunOTOSInternal raw = new SparkFunOTOSInternal();
        raw.update();
        HardwareMap hardware = new HardwareMap();
        hardware.put("otos", new StarterBotOTOS(raw, RobotProfile.loadActive("starterbot")));
        hardware.setActive(true);
        TrackingSubsystem tracking = new TrackingSubsystem();
        tracking.init(hardware, RobotProfile.loadActive("starterbot"));
        tracking.start(new FieldMap.Pose(0, 0, 0));
        odo.update(pose(0, 10, 0), zero, zero);
        raw.update();
        tracking.update(0);
        assertEquals(10, tracking.getPose().x, 1e-8);
        assertEquals(0, tracking.getPose().y, 1e-8);
        odo.update(pose(2, 10, 0), zero, zero);
        raw.update();
        tracking.update(0);
        assertEquals(10, tracking.getPose().x, 1e-8);
        assertEquals(-2, tracking.getPose().y, 1e-8);
        odo.update(pose(2, 10, -90), zero, zero);
        raw.update();
        tracking.update(-90);
        assertEquals(10, tracking.getPose().x, 1e-8);
        assertEquals(-2, tracking.getPose().y, 1e-8);
        odo.update(pose(2, 10, 90), zero, zero);
        raw.update();
        tracking.update(90);
        assertEquals(10, tracking.getPose().x, 1e-8);
        assertEquals(-2, tracking.getPose().y, 1e-8);
    }

    @Test public void otosIndividualReadingsUseSameDataAsBurstRead() {
        CommonOdometry odo = CommonOdometry.getInstance();
        odo.update(pose(0, 0, 0), pose(0, 0, 0), pose(0, 0, 0));
        SparkFunOTOSInternal raw = new SparkFunOTOSInternal();
        raw.update();
        StarterBotOTOS otos = new StarterBotOTOS(raw, RobotProfile.loadActive("starterbot"));
        otos.resetTracking();
        odo.update(pose(2, 10, 90), pose(3, 4, 20), pose(5, 6, 30));
        raw.update();
        SparkFunOTOS.Pose2D p = new SparkFunOTOS.Pose2D();
        SparkFunOTOS.Pose2D v = new SparkFunOTOS.Pose2D();
        SparkFunOTOS.Pose2D a = new SparkFunOTOS.Pose2D();
        otos.getPosVelAcc(p, v, a);
        assertEquals(p.x, otos.getPosition().x, 1e-8);
        assertEquals(p.y, otos.getPosition().y, 1e-8);
        assertEquals(v.x, otos.getVelocity().x, 1e-8);
        assertEquals(v.y, otos.getVelocity().y, 1e-8);
        assertEquals(a.x, otos.getAcceleration().x, 1e-8);
        assertEquals(a.y, otos.getAcceleration().y, 1e-8);
        assertTrue(otos.setLinearScalar(1.05));
        assertEquals(1.05, otos.getLinearScalar(), 0);
        assertThrows(UnsupportedOperationException.class, () -> otos.setPosition(p));
    }

    @Test public void fieldTrackingOpModeInitializesAndRunsWithSimulatedHardware() throws Exception {
        CommonOdometry odo = CommonOdometry.getInstance();
        Pose2D zero = pose(0, 0, 0);
        odo.update(zero, zero, zero);
        SparkFunOTOSInternal raw = new SparkFunOTOSInternal();
        raw.update();
        HardwareMap hardware = new HardwareMap();
        for (String name : new String[] {"left_front_drive", "right_front_drive",
                "left_back_drive", "right_back_drive"}) {
            hardware.put(name, mock(DcMotor.class));
        }
        IMU imu = mock(IMU.class);
        when(imu.initialize(any())).thenReturn(true);
        when(imu.getRobotYawPitchRollAngles()).thenReturn(
                new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0));
        when(imu.getRobotAngularVelocity(AngleUnit.DEGREES)).thenReturn(
                new org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity(
                        AngleUnit.DEGREES, 0, 0, 0, 0));
        hardware.put("imu", imu);
        hardware.put("otos", new StarterBotOTOS(raw, RobotProfile.loadActive("starterbot")));
        hardware.put("limelight", new Limelight3A());
        hardware.setActive(true);
        VirtualRobotController controller = mock(VirtualRobotController.class);
        java.lang.reflect.Field mapField = VirtualRobotController.class.getDeclaredField("hardwareMap");
        mapField.setAccessible(true);
        mapField.set(controller, hardware);
        for (String fieldName : new String[] {"gamePad1", "gamePad2"}) {
            java.lang.reflect.Field gamepadField = VirtualRobotController.class.getDeclaredField(fieldName);
            gamepadField.setAccessible(true);
            gamepadField.set(controller, new Gamepad());
        }
        OpMode.setVirtualRobotController(controller);
        FieldTrackingTestTeleOp op = new FieldTrackingTestTeleOp();
        op.telemetry = mock(Telemetry.class);
        op.init();
        op.init_loop();
        op.start();
        op.loop();
        verify(op.telemetry, atLeastOnce()).addLine("Configuration");
        verify(op.telemetry, atLeastOnce()).addLine("Field tracking");
        verify(op.telemetry, atLeastOnce()).addLine(contains("Estimated; OTOS + IMU"));
        verify(op.telemetry, atLeastOnce()).addLine(contains("Searching; not fused"));
        op.stop();

        LandmarkTravelTestTeleOp travel = new LandmarkTravelTestTeleOp();
        travel.telemetry = mock(Telemetry.class);
        travel.init();
        travel.init_loop();
        travel.gamepad1.a = true;
        travel.start();
        travel.loop();
        verify(travel.telemetry, atLeastOnce()).addData("Navigation status", "Ready");
        verify(travel.telemetry, atLeastOnce()).addLine("> West flower");
        verify(travel.telemetry, atLeastOnce()).addLine("  Blue scoring shooting start");
        travel.gamepad1.a = false;
        travel.gamepad1.dpad_down = true;
        travel.loop();
        verify(travel.telemetry, atLeastOnce()).addLine("> East flower");
        travel.gamepad1.dpad_down = false;
        travel.gamepad1.left_stick_y = -1;
        travel.loop();
        verify(travel.telemetry, atLeastOnce()).addData("Control status", "Manual drive at 50%");
        travel.gamepad1.y = true;
        travel.loop();
        verify(travel.telemetry, atLeastOnce()).addData("Control status", "Stopped by Y");
        travel.stop();
    }

    @Test public void pedroNavigationDrivesSimulatedRobotToLandmarkWithoutVision() throws Exception {
        Assume.assumeTrue("A graphical display is required",
                System.getenv("DISPLAY") != null || System.getProperty("os.name").startsWith("Windows")
                        || System.getProperty("os.name").startsWith("Mac"));
        Platform.startup(() -> { });
        try {
            for (String model : new String[] {"biobuzz_starterbot", "biobuzz_competitionbot"}) {
                assertSimulatedTrips(model);
            }
        } finally {
            Platform.exit();
        }
    }

    private void assertSimulatedTrips(String model) throws Exception {
        VirtualRobotController controller = new VirtualRobotController();
        java.lang.reflect.Field pane = VirtualRobotController.class.getDeclaredField("fieldPane");
        pane.setAccessible(true);
        pane.set(controller, new Pane());
        java.lang.reflect.Field halfWidth = VirtualRobotController.class.getDeclaredField("halfFieldWidth");
        halfWidth.setAccessible(true);
        halfWidth.setDouble(controller, 324);
        java.lang.reflect.Method setController = VirtualBot.class.getDeclaredMethod(
                "setController", VirtualRobotController.class);
        setController.setAccessible(true);
        setController.invoke(null, controller);

        FXMLLoader loader = new FXMLLoader(getClass().getResource(
                "/virtual_robot/robots/fxml/" + model + ".fxml"));
        loader.load();
        VirtualBot robot = loader.getController();
        HardwareMap hardware = robot.getHardwareMap();
        hardware.setActive(true);
        FieldMap.Pose start = new FieldMap.Pose(50, -45, 0);
        java.lang.reflect.Field bodyField = VirtualBot.class.getDeclaredField("chassisBody");
        bodyField.setAccessible(true);
        Body body = (Body) bodyField.get(robot);
        org.dyn4j.geometry.Transform transform = new org.dyn4j.geometry.Transform();
        transform.rotate(Math.toRadians(start.headingDegrees - 90));
        transform.translate(start.x * 0.0254, start.y * 0.0254);
        body.setTransform(transform);
        body.setLinearVelocity(0, 0);
        body.setAngularVelocity(0);
        robot.updateStateAndSensors(20);

        DriveSubsystem drive = new DriveSubsystem();
        RobotProfile profile = RobotProfile.loadActive(null);
        drive.init(hardware, profile);
        drive.resetHeading(start.headingDegrees);
        TrackingSubsystem tracking = new TrackingSubsystem();
        tracking.init(hardware, profile);
        LandmarkFieldMap map = new LandmarkFieldMap(profile);
        LandmarkNavigationService navigation = new LandmarkNavigationService(drive, tracking, profile);
        tracking.start(start);
        assertTrue(profile.isSimulatorRuntime());
        assertTrue(profile.isAutonomousReady());
        double initialX = robot.getX();
        double initialY = robot.getY();
        World<Body> world = controller.getWorld();
        world.setGravity(World.ZERO_GRAVITY);
        boolean sawNoVision = hardware.get(Limelight3A.class, "limelight")
                .getLatestResult().getFiducialResults().isEmpty();
        java.util.List<LandmarkFieldMap.Destination> targets = new java.util.ArrayList<>();
        targets.add(LandmarkFieldMap.Destination.savedHome(new FieldMap.Pose(55, -45, 0)));
        targets.addAll(map.destinations());
        int tick = 0;
        for (LandmarkFieldMap.Destination target : targets) {
            assertTrue(model + " -> " + target.name + ": " + navigation.getStatus(),
                    navigation.start(target, tick * 0.02));
            int startedTick = tick;
            while (tick - startedTick < 3000 && navigation.isActive()) {
                tick++;
                world.updatev(0.02);
                robot.updateStateAndSensors(20);
                // The simulated IMU is wall-time throttled; use measured body motion for this physics-time loop.
                tracking.update(Math.toDegrees(body.getTransform().getRotationAngle()) + 90,
                        Math.toDegrees(body.getAngularVelocity()));
                assertEquals("Tracked field X velocity", body.getLinearVelocity().x / 0.0254,
                        tracking.getVelocityX(), 0.01);
                assertEquals("Tracked field Y velocity", body.getLinearVelocity().y / 0.0254,
                        tracking.getVelocityY(), 0.01);
                navigation.update(tick * 0.02);
                for (double power : new double[] {drive.getLeftFrontPower(), drive.getRightFrontPower(),
                        drive.getLeftBackPower(), drive.getRightBackPower()}) {
                    assertTrue("Each wheel must respect the reported power limit",
                            Math.abs(power) <= navigation.getPowerLimit() + 1e-9);
                }
                sawNoVision &= hardware.get(Limelight3A.class, "limelight")
                        .getLatestResult().getFiducialResults().isEmpty();
            }
            assertTrue("No target vision was used or fabricated", sawNoVision);
            assertTrue("Simulator robot traveled; nav=" + navigation.getStatus()
                    + ", tracked=" + tracking.getPose() + ", actual="
                    + robot.getX() + "," + robot.getY(), Math.hypot(
                    robot.getX() - initialX, robot.getY() - initialY) > 5);
            assertFalse("Navigation stopped with telemetry failure: " + navigation.getStatus(),
                    navigation.isActive());
            assertTrue("Expected arrival; status=" + navigation.getStatus()
                            + " model=" + model + " target=" + target.name
                            + " pose=" + tracking.getPose().x + "," + tracking.getPose().y
                            + " heading=" + tracking.getPose().headingDegrees
                            + " speed=" + tracking.getSpeedInchesPerSecond(),
                    navigation.getStatus().startsWith("Arrived"));
            double pixelsPerInch = VirtualField.PIXELS_PER_METER * 0.0254;
            assertTrue("Actual robot center must arrive within one inch",
                    Math.hypot(target.robotPose.x - robot.getX() / pixelsPerInch,
                            target.robotPose.y - robot.getY() / pixelsPerInch) <= 1.0);
            assertTrue("Measured translation must be stationary",
                    tracking.getSpeedInchesPerSecond() <= profile.navigation.stationarySpeedInPerSec);
            assertTrue("Actual heading must arrive within three degrees",
                    Math.abs(Math.toDegrees(body.getTransform().getRotationAngle()) + 90
                            - target.robotPose.headingDegrees
                            - 360 * Math.floor((Math.toDegrees(body.getTransform().getRotationAngle()) + 90
                            - target.robotPose.headingDegrees + 180) / 360)) <= 3.0);
            assertEquals(0, drive.getLeftFrontPower(), 0);
            assertEquals(0, drive.getRightFrontPower(), 0);
            assertEquals(0, drive.getLeftBackPower(), 0);
            assertEquals(0, drive.getRightBackPower(), 0);
        }
    }
}
