package local_simulator_config;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import com.qualcomm.hardware.CommonOdometry;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.hardware.sparkfun.SparkFunOTOSInternal;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.math.Pose;
import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.pedropathing.revhub.localizers.OTOSLocalizer;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.teamcode.control.BioBuzzFieldMap;
import org.firstinspires.ftc.teamcode.BioBuzzLimelightClusterAdapter.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.ConfigurationSubsystem.StartingSpot;
import org.firstinspires.ftc.teamcode.subsystems.FieldMap;
import org.firstinspires.ftc.teamcode.pedro.PedroConstants;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.LandmarkTravelTestTeleOp;
import org.firstinspires.ftc.teamcode.subsystems.RobotProfile;
import org.firstinspires.ftc.teamcode.subsystems.ActiveRobotProfileSource;
import org.junit.Before;
import org.junit.Assume;
import org.junit.Test;
import virtual_robot.controller.VirtualRobotController;
import virtual_robot.controller.VirtualBot;
import virtual_robot.controller.Game;
import virtual_robot.games.BioBuzz;
import javafx.application.Platform;
import javafx.fxml.FXMLLoader;
import javafx.scene.layout.Pane;
import org.dyn4j.dynamics.Body;
import org.dyn4j.geometry.Transform;
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

    @Test public void pedroLocalizerCanSetPoseAndTrackTranslation() {
        CommonOdometry odo = CommonOdometry.getInstance();
        Pose2D zero = pose(0, 0, 0);
        odo.update(zero, zero, zero);
        SparkFunOTOSInternal raw = new SparkFunOTOSInternal();
        raw.update();
        HardwareMap hardware = new HardwareMap();
        hardware.put("otos", new StarterBotOTOS(raw, RobotProfile.loadActive("starterbot")));
        hardware.setActive(true);
        OTOSLocalizer localizer = PedroConstants.localizer(hardware);
        localizer.setPose(new Pose(10, 20, 0));
        localizer.update();
        assertEquals(10, localizer.pose().x(), 1e-8);
        assertEquals(20, localizer.pose().y(), 1e-8);
        assertEquals(0, localizer.pose().heading(), 1e-8);
        odo.update(pose(0, 10, 0), pose(0, 4, 0), zero);
        raw.update();
        localizer.update();
        assertEquals(20, localizer.pose().x(), 1e-8);
        assertEquals(20, localizer.pose().y(), 1e-8);
        assertEquals(4, localizer.velocity().vx, 1e-8);
        assertEquals(0, localizer.velocity().vy, 1e-8);

        odo.update(pose(0, 10, 90), pose(0, 4, 30), zero);
        raw.update();
        localizer.update();
        assertEquals(Math.PI / 2, localizer.pose().heading(), 1e-8);
        assertEquals(4, localizer.velocity().vx, 1e-8);
        assertEquals(0, localizer.velocity().vy, 1e-8);
        assertEquals(Math.PI / 6, localizer.velocity().omega, 1e-8);
        localizer.setPose(new Pose(-5, 6, Math.PI));
        localizer.update();
        assertEquals(-5, localizer.pose().x(), 1e-8);
        assertEquals(6, localizer.pose().y(), 1e-8);
        assertEquals(Math.PI, localizer.pose().heading(), 1e-8);
    }

    @Test public void pedroFollowerMovesBothSimulatedRobots() throws Exception {
        Assume.assumeTrue("A graphical display is required",
                System.getenv("DISPLAY") != null || System.getProperty("os.name").startsWith("Windows")
                        || System.getProperty("os.name").startsWith("Mac"));
        Platform.startup(() -> { });
        try {
            for (String model : new String[] {"biobuzz_starterbot", "biobuzz_competitionbot"}) {
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
                robot.updateStateAndSensors(20);
                java.lang.reflect.Field bodyField = VirtualBot.class.getDeclaredField("chassisBody");
                bodyField.setAccessible(true);
                Body body = (Body) bodyField.get(robot);
                double initialX = body.getTransform().getTranslationX() / 0.0254;
                double initialY = body.getTransform().getTranslationY() / 0.0254;
                Follower follower = PedroConstants.create(hardware);
                follower.setPose(new Pose(0, 0, Math.PI / 2));
                Pose start = follower.pose();
                Pose end = new Pose(0, 12, Math.PI / 2);
                follower.follow(Paths.line(start, end).constant(end));
                World<Body> world = controller.getWorld();
                world.setGravity(World.ZERO_GRAVITY);
                for (int tick = 0; tick < 1000 && follower.following(); tick++) {
                    follower.update(0.02);
                    world.updatev(0.02);
                    robot.updateStateAndSensors(20);
                    double traveled = body.getTransform().getTranslationY() / 0.0254 - initialY;
                    if (traveled > 1 && traveled < 12.5) {
                        assertTrue(model + " forward path reverses at " + traveled
                                        + " with speed " + body.getLinearVelocity().y / 0.0254,
                                body.getLinearVelocity().y / 0.0254 > -1);
                    }
                }
                assertEquals(model + " endpoint", 12, follower.pose().y(), 2);
                assertEquals(model + " sideways drift", 0, follower.pose().x(), 2);
                assertEquals(model + " actual forward", 12,
                        body.getTransform().getTranslationY() / 0.0254 - initialY, 2);
                assertEquals(model + " actual sideways", 0,
                        body.getTransform().getTranslationX() / 0.0254 - initialX, 2);
                assertSettledHold(model + " forward", follower, robot, body, world);

                start = follower.pose();
                end = new Pose(12, 12, Math.PI / 2);
                follower.follow(Paths.line(start, end).constant(end));
                for (int tick = 0; tick < 1000 && follower.following(); tick++) {
                    follower.update(0.02);
                    world.updatev(0.02);
                    robot.updateStateAndSensors(20);
                }
                assertEquals(model + " strafe endpoint", 12, follower.pose().x(), 2);
                assertEquals(model + " actual strafe", 12,
                        body.getTransform().getTranslationX() / 0.0254 - initialX, 2);
                assertSettledHold(model + " strafe", follower, robot, body, world);

                start = follower.pose();
                end = new Pose(12, 18, 0);
                follower.follow(Paths.line(start, end).linear(start, end));
                for (int tick = 0; tick < 1000 && follower.following(); tick++) {
                    follower.update(0.02);
                    world.updatev(0.02);
                    robot.updateStateAndSensors(20);
                }
                for (int tick = 0; tick < 100 && follower.holding(); tick++) {
                    follower.update(0.02);
                    world.updatev(0.02);
                    robot.updateStateAndSensors(20);
                }
                assertEquals(model + " turning endpoint", 18, follower.pose().y(), 2);
                assertEquals(model + " actual turning endpoint", 18,
                        body.getTransform().getTranslationY() / 0.0254 - initialY, 2);
                assertEquals(model + " heading", 0,
                        Math.atan2(Math.sin(follower.pose().heading()),
                                Math.cos(follower.pose().heading())), 0.15);
                assertEquals(model + " actual heading", 0,
                        Math.atan2(Math.sin(body.getTransform().getRotationAngle() + Math.PI / 2),
                                Math.cos(body.getTransform().getRotationAngle() + Math.PI / 2)), 0.15);
                assertSettledHold(model + " turn", follower, robot, body, world);
                assertHoldWithOffsetTiming(model, follower, robot, body, world);
                follower.drivetrain.stop();

                java.lang.reflect.Field mapField = VirtualRobotController.class.getDeclaredField("hardwareMap");
                mapField.setAccessible(true);
                mapField.set(controller, hardware);
                OpMode.setVirtualRobotController(controller);
                LandmarkTravelTestTeleOp op = new LandmarkTravelTestTeleOp();
                op.telemetry = mock(Telemetry.class);
                op.init();
                op.init_loop();
                op.start();
                for (BioBuzzFieldMap.Destination destination : new BioBuzzFieldMap(18, 18).destinations()) {
                    verify(op.telemetry, atLeastOnce()).addLine(contains(destination.name));
                }
                op.gamepad1.options = true;
                op.loop();
                op.gamepad1.options = false;
                body.setLinearVelocity(0, 0.2);
                robot.updateStateAndSensors(20);
                op.loop();
                op.gamepad1.x = true;
                op.loop();
                verify(op.telemetry, atLeastOnce()).addLine("Calibration refused: tracking must be");
                op.gamepad1.x = false;
                body.setLinearVelocity(0, 0);
                body.setAngularVelocity(0);
                body.clearAccumulatedForce();
                body.clearAccumulatedTorque();
                world.updatev(0.02);
                robot.updateStateAndSensors(20);
                op.loop();
                op.gamepad1.x = true;
                op.loop();
                verify(op.telemetry, atLeastOnce()).addLine("OTOS IMU calibrated; field coordinates");
                op.gamepad1.x = false;
                op.gamepad1.dpad_up = true;
                op.loop();
                verify(op.telemetry, atLeastOnce()).addLine("> Saved home");
                op.gamepad1.dpad_up = false;
                op.gamepad1.dpad_down = true;
                op.loop();
                op.gamepad1.dpad_down = false;
                op.loop();
                java.lang.reflect.Field followerField =
                        LandmarkTravelTestTeleOp.class.getDeclaredField("follower");
                followerField.setAccessible(true);
                Follower landmarkFollower = (Follower) followerField.get(op);
                if (landmarkFollower.pose().y() < 0) {
                    op.gamepad1.dpad_down = true;
                    op.loop();
                    op.gamepad1.dpad_down = false;
                    op.loop();
                }
                op.gamepad1.a = true;
                op.loop();
                assertTrue(model + " landmark route started", landmarkFollower.following());
                op.gamepad1.a = false;
                op.gamepad1.b = true;
                op.loop();
                assertFalse(model + " landmark route stopped", landmarkFollower.following());
                op.stop();

                Game.setController(controller);
                new BioBuzz().initialize();
                List<Body> obstacles = new ArrayList<>(world.getBodies());
                obstacles.remove(body);
                java.lang.reflect.Method setupWorld =
                        VirtualRobotController.class.getDeclaredMethod("setupPhysicsWorld");
                setupWorld.setAccessible(true);
                setupWorld.invoke(controller);
                RobotProfile profile = RobotProfile.loadActive(null);
                BioBuzzFieldMap map = new BioBuzzFieldMap(
                        profile.chassisLengthCm / 2.54, profile.chassisWidthCm / 2.54);
                for (Alliance alliance : new Alliance[] {Alliance.RED, Alliance.BLUE}) {
                    for (StartingSpot side : StartingSpot.values()) {
                        FieldMap.Pose routeStart = map.startingPose(alliance, side);
                        for (BioBuzzFieldMap.Destination destination : map.destinations()) {
                            String label = model + " " + alliance + " " + side + " to " + destination.name;
                            assertSimulatedRoute(label, map, routeStart, destination.pose,
                                    follower, robot, body, world, obstacles);
                        }
                    }
                }
                follower.drivetrain.stop();
                Transform collisionProbe = new Transform();
                collisionProbe.translate(24 * 0.0254, -68 * 0.0254);
                body.setTransform(collisionProbe);
                body.setLinearVelocity(0, 0);
                body.setAngularVelocity(0);
                world.updatev(0.02);
                assertTrue(model + " flower contact detection",
                        obstacles.stream().anyMatch(obstacle -> world.isInContact(body, obstacle)));
            }
        } finally {
            Platform.exit();
        }
    }

    private static void assertSettledHold(String label, Follower follower, VirtualBot robot,
            Body body, World<Body> world) {
        assertTrue(label + " enters hold", follower.holding());
        for (int tick = 0; tick < 200; tick++) {
            follower.update(0.02);
            world.updatev(0.02);
            robot.updateStateAndSensors(20);
            if (tick >= 100) {
                assertTrue(label + " hold linear speed " + body.getLinearVelocity().getMagnitude() / 0.0254,
                        body.getLinearVelocity().getMagnitude() / 0.0254 < 0.05);
                assertTrue(label + " hold angular speed",
                        Math.abs(body.getAngularVelocity()) < 0.05);
            }
        }
    }

    private static void assertHoldWithOffsetTiming(String label, Follower follower, VirtualBot robot,
            Body body, World<Body> world) {
        Pose start = follower.pose();
        Pose end = new Pose(start.x(), start.y() + 24, start.heading());
        follower.follow(Paths.line(start, end).constant(end));
        double[] loopIntervals = {0.016, 0.026, 0.020, 0.024, 0.018};
        double nextPhysics = 0.02;
        double nextLoop = 0;
        double previousLoop = 0;
        double holdStart = -1;
        int loops = 0;
        while (nextPhysics < 15) {
            if (nextLoop <= nextPhysics) {
                follower.update(nextLoop - previousLoop);
                previousLoop = nextLoop;
                nextLoop += loopIntervals[loops++ % loopIntervals.length];
                if (follower.holding() && holdStart < 0) holdStart = previousLoop;
            } else {
                world.updatev(0.02);
                robot.updateStateAndSensors(20);
                if (holdStart >= 0 && nextPhysics >= holdStart + 2) {
                    assertTrue(label + " offset-timing hold linear speed",
                            body.getLinearVelocity().getMagnitude() / 0.0254 < 0.05);
                    assertTrue(label + " offset-timing hold angular speed",
                            Math.abs(body.getAngularVelocity()) < 0.05);
                }
                nextPhysics += 0.02;
            }
        }
        assertTrue(label + " offset-timing path reached hold", holdStart >= 0 && holdStart < 13);
        assertEquals(label + " offset-timing endpoint x", end.x(), follower.pose().x(), 2);
        assertEquals(label + " offset-timing endpoint y", end.y(), follower.pose().y(), 2);
    }

    private static void assertSimulatedRoute(String label, BioBuzzFieldMap map,
            FieldMap.Pose start, FieldMap.Pose goal, Follower follower, VirtualBot robot,
            Body body, World<Body> world, List<Body> obstacles) {
        List<FieldMap.Pose> waypoints = map.plan(start, goal);
        if (waypoints.isEmpty()) return;
        follower.stop();
        follower.drivetrain.stop();
        Transform transform = new Transform();
        transform.rotate(Math.toRadians(start.headingDegrees - 90));
        transform.translate(start.x * 0.0254, start.y * 0.0254);
        body.setTransform(transform);
        body.setLinearVelocity(0, 0);
        body.setAngularVelocity(0);
        robot.updateStateAndSensors(20);
        Pose from = new Pose(start.x, start.y, Math.toRadians(start.headingDegrees));
        follower.setPose(from);
        List<Path> segments = new ArrayList<>();
        for (FieldMap.Pose waypoint : waypoints) {
            Pose to = new Pose(waypoint.x, waypoint.y, Math.toRadians(waypoint.headingDegrees));
            segments.add(Paths.line(from, to).linear(from, to));
            from = to;
        }
        follower.follow(Paths.path(segments.toArray(new Path[0])));
        for (int tick = 0; tick < 6000 && follower.following(); tick++) {
            follower.update(0.02);
            world.updatev(0.02);
            robot.updateStateAndSensors(20);
            for (Body obstacle : obstacles) {
                assertFalse(label + " collided with field geometry at "
                        + body.getTransform().getTranslation() + " velocity "
                        + body.getLinearVelocity(), world.isInContact(body, obstacle));
            }
        }
        assertFalse(label + " route did not finish", follower.following());
        assertTrue(label + " enters hold", follower.holding());
        for (int tick = 0; tick < 200; tick++) {
            follower.update(0.02);
            world.updatev(0.02);
            robot.updateStateAndSensors(20);
            for (Body obstacle : obstacles) {
                assertFalse(label + " collided while settling", world.isInContact(body, obstacle));
            }
            if (tick >= 100) {
                assertTrue(label + " hold linear speed " + body.getLinearVelocity().getMagnitude() / 0.0254,
                        body.getLinearVelocity().getMagnitude() / 0.0254 < 0.05);
                assertTrue(label + " hold angular speed " + body.getAngularVelocity(),
                        Math.abs(body.getAngularVelocity()) < 0.05);
            }
        }
        assertEquals(label + " actual landmark x", goal.x,
                body.getTransform().getTranslationX() / 0.0254, 2);
        assertEquals(label + " actual landmark y", goal.y,
                body.getTransform().getTranslationY() / 0.0254, 2);
        double headingError = body.getTransform().getRotationAngle() + Math.PI / 2
                - Math.toRadians(goal.headingDegrees);
        assertEquals(label + " actual landmark heading", 0,
                Math.atan2(Math.sin(headingError), Math.cos(headingError)), 0.2);
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

}
