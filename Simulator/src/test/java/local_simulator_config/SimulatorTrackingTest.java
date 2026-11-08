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
import org.firstinspires.ftc.teamcode.subsystems.FieldMap;
import org.firstinspires.ftc.teamcode.subsystems.RobotProfile;
import org.firstinspires.ftc.teamcode.subsystems.TrackingSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ActiveRobotProfileSource;
import org.junit.Before;
import org.junit.Test;
import virtual_robot.controller.VirtualRobotController;

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
    }
}
