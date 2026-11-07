package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.*;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.control.OTOSFieldNavigator;
import org.junit.Before;
import org.junit.After;
import org.junit.Test;
import org.mockito.MockedConstruction;
import org.mockito.MockedStatic;
import org.firstinspires.ftc.teamcode.subsystems.RobotProfile;
import org.junit.Rule;
import org.junit.rules.TemporaryFolder;
import java.io.File;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;

public class OTOSFieldNavigationTestTeleOpTest {
    private static final double SENSOR_BACK = 9 - 19 / 2.54;
    private static final double SENSOR_RIGHT = 7.5 / 2.54 - 9;
    private OTOSFieldNavigationTestTeleOp opMode;
    private SparkFunOTOS otos;
    private SparkFunOTOS.Status status;
    private final SparkFunOTOS.Pose2D pose = new SparkFunOTOS.Pose2D();
    private final SparkFunOTOS.Pose2D speed = new SparkFunOTOS.Pose2D();
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private double time;
    private MockedConstruction<RevHubOrientationOnRobot> orientation;
    private MockedStatic<RobotProfile> profiles;
    @Rule public TemporaryFolder folder = new TemporaryFolder();

    @Before
    public void setUp() {
        RobotProfile profile = TestProfiles.starter();
        profiles = mockStatic(RobotProfile.class);
        profiles.when(() -> RobotProfile.loadActive("starterbot")).thenReturn(profile);
        orientation = mockConstruction(RevHubOrientationOnRobot.class);
        opMode = spy(new OTOSFieldNavigationTestTeleOp());
        doAnswer(invocation -> time).when(opMode).getRuntime();
        opMode.hardwareMap = mock(HardwareMap.class);
        opMode.gamepad1 = mock(Gamepad.class);
        opMode.telemetry = mock(Telemetry.class);
        frontLeft = motor("left_front_drive");
        frontRight = motor("right_front_drive");
        backLeft = motor("left_back_drive");
        backRight = motor("right_back_drive");
        IMU imu = mock(IMU.class);
        when(opMode.hardwareMap.get(IMU.class, "imu")).thenReturn(imu);
        when(imu.initialize(any())).thenReturn(true);
        otos = mock(SparkFunOTOS.class);
        when(opMode.hardwareMap.get(SparkFunOTOS.class, "otos")).thenReturn(otos);
        when(otos.calibrateImu()).thenReturn(true);
        status = new SparkFunOTOS.Status();
        when(otos.getStatus()).thenReturn(status);
        doAnswer(invocation -> {
            SparkFunOTOS.Pose2D position = invocation.getArgument(0);
            position.x = pose.x;
            position.y = pose.y;
            position.h = pose.h;
            SparkFunOTOS.Pose2D velocity = invocation.getArgument(1);
            velocity.x = speed.x;
            velocity.y = speed.y;
            velocity.h = speed.h;
            return null;
        }).when(otos).getPosVelAcc(any(), any(), any());
    }

    @After
    public void tearDown() {
        orientation.close();
        profiles.close();
    }

    @Test
    public void profileControlsSensorNameAndScalarsWhileKeepingSensorOffsetZero() throws Exception {
        String json = new String(Files.readAllBytes(
                new File("../robot-configs/biobuzz-starterbot.profile.json").toPath()),
                StandardCharsets.UTF_8).replace("\"name\": \"otos\"", "\"name\": \"other_otos\"")
                .replace("\"linearScalar\": 1.0", "\"linearScalar\": 1.02")
                .replace("\"angularScalar\": 1.0", "\"angularScalar\": 0.98");
        Files.write(new File(folder.getRoot(), "alternate.profile.json").toPath(),
                json.getBytes(StandardCharsets.UTF_8));
        profiles.close();
        RobotProfile alternate = RobotProfile.load(folder.getRoot(), "alternate", "starterbot");
        profiles = mockStatic(RobotProfile.class);
        profiles.when(() -> RobotProfile.loadActive("starterbot")).thenReturn(alternate);
        when(opMode.hardwareMap.get(SparkFunOTOS.class, "other_otos")).thenReturn(otos);
        opMode.init();
        verify(opMode.hardwareMap).get(SparkFunOTOS.class, "other_otos");
        verify(otos).setLinearScalar(1.02);
        verify(otos).setAngularScalar(0.98);
        org.mockito.ArgumentCaptor<SparkFunOTOS.Pose2D> offset =
                org.mockito.ArgumentCaptor.forClass(SparkFunOTOS.Pose2D.class);
        verify(otos).setOffset(offset.capture());
        org.junit.Assert.assertEquals(0, offset.getValue().x, 0);
        org.junit.Assert.assertEquals(0, offset.getValue().y, 0);
        org.junit.Assert.assertEquals(0, offset.getValue().h, 0);
    }

    @Test
    public void initializesDriveAndSensorAndPublishesLiveReadingsBeforeAndAfterStart() {
        pose.x = 12.5;
        pose.y = -3.0;
        pose.h = 90.0;
        status.warnOpticalTracking = true;
        opMode.init();
        verify(opMode.hardwareMap).get(SparkFunOTOS.class, "otos");
        verify(otos).setLinearUnit(DistanceUnit.INCH);
        verify(otos).setAngularUnit(AngleUnit.DEGREES);
        verify(otos).setLinearScalar(1.0);
        verify(otos).setAngularScalar(1.0);
        verify(otos).setOffset(any(SparkFunOTOS.Pose2D.class));
        verify(otos).calibrateImu();
        verify(otos).resetTracking();

        opMode.init_loop();
        opMode.loop();

        verify(otos, times(3)).getPosVelAcc(any(), any(), any());
        verify(opMode.telemetry, times(3))
                .addData("Position (in)", "X %.2f, Y %.2f", 12.5, -3.0);
        verify(opMode.telemetry, times(3)).addData("Heading (deg)", "%.2f", 90.0);
        verify(opMode.telemetry, times(3)).addData("Optical tracking warning", true);
    }

    @Test
    public void saveHomeAndCalibrationRunOncePerPressWithoutResettingFieldCoordinates() {
        opMode.init();
        opMode.gamepad1.x = true;
        opMode.gamepad1.options = true;
        opMode.init_loop();
        opMode.loop();
        opMode.loop();
        verify(otos, times(2)).calibrateImu();
        verify(otos, times(1)).resetTracking();
        verify(opMode.telemetry, times(3)).addData("Drive status",
                "Red-wall home saved; final approach stops short of wall");

        opMode.gamepad1.x = false;
        opMode.gamepad1.options = false;
        opMode.loop();
        opMode.gamepad1.x = true;
        opMode.gamepad1.options = true;
        opMode.loop();
        verify(otos, times(3)).calibrateImu();
        verify(otos, times(1)).resetTracking();
    }

    @Test
    public void yStopsWithoutSavingHomeAndStartAliasSavesOnce() {
        startReturn();
        opMode.gamepad1.y = true;
        opMode.gamepad1.options = true;
        opMode.gamepad1.left_stick_y = -1;
        clearMotors();
        opMode.loop();
        assertPowers(0, 0, 0, 0);
        verify(opMode.telemetry).addData("Drive status", "Stopped by Y");
        verify(opMode.telemetry, never()).addData("Drive status",
                "Red-wall home saved; final approach stops short of wall");
        opMode.gamepad1.y = false;
        opMode.gamepad1.options = false;
        opMode.gamepad1.left_stick_y = 0;
        opMode.loop();
        opMode.gamepad1.start = true;
        opMode.loop();
        opMode.loop();
        verify(opMode.telemetry, times(2)).addData("Drive status",
                "Home saved; field coordinates preserved");
        verify(otos).resetTracking();
    }

    @Test
    public void calibrationFailureIsNotReportedAsSuccessfulInitialization() {
        when(otos.calibrateImu()).thenReturn(false);
        assertThrows(IllegalStateException.class, () -> opMode.init());
        verify(otos, never()).resetTracking();
        verify(otos, never()).getPosVelAcc(any(), any(), any());
    }

    @Test
    public void manualDrivingIsDisabledDuringInitAndLimitedAfterStart() {
        opMode.init();
        clearMotors();
        opMode.gamepad1.left_stick_y = -1;
        opMode.init_loop();
        assertPowers(0, 0, 0, 0);
        clearMotors();
        opMode.loop();
        assertPowers(0.45, 0.45, 0.45, 0.45);
        clearMotors();
        opMode.stop();
        assertPowers(0, 0, 0, 0);
    }

    @Test
    public void manualDrivingSendsOnePowerCommandPerMotorWithoutStopPulses() {
        opMode.init();
        opMode.gamepad1.left_stick_y = -1;
        for (int i = 0; i < 3; i++) {
            clearMotors();
            opMode.loop();
            for (DcMotor motor : new DcMotor[] {frontLeft, frontRight, backLeft, backRight}) {
                verify(motor).setPower(0.45);
                verifyNoMoreInteractions(motor);
            }
        }
    }

    @Test
    public void stickOverrideSwitchesDirectlyFromReturnToManualWithoutStopPulse() {
        startReturn();
        opMode.gamepad1.left_stick_y = -1;
        clearMotors();
        opMode.loop();
        for (DcMotor motor : new DcMotor[] {frontLeft, frontRight, backLeft, backRight}) {
            verify(motor).setPower(0.45);
            verifyNoMoreInteractions(motor);
        }
        opMode.gamepad1.left_stick_y = 0;
        clearMotors();
        opMode.loop();
        assertPowers(0, 0, 0, 0);
    }

    @Test
    public void returnsFromForwardDisplacementUsingMeasuredAxesAndAvoidsFrameFromBlueSide() {
        opMode.init();
        pose.x = -12;
        opMode.gamepad1.a = true;
        opMode.loop();
        assertLatestPowerSigns(-1, -1, -1, -1);

        opMode.gamepad1.b = true;
        opMode.loop();
        opMode.gamepad1.b = false;
        opMode.gamepad1.a = false;
        opMode.loop();
        setRobotPose(45, 0, 0);
        opMode.gamepad1.a = true;
        clearMotors();
        opMode.loop();
        verify(opMode.telemetry).addData("Return waypoints remaining", 3);
        verify(opMode.telemetry, atLeastOnce()).addData("Drive status", "Returning: mapped waypoints");
        assertTrue(Math.abs(latestPower(frontLeft)) <= 0.45);
    }

    @Test
    public void rotatesTranslationIntoRobotFrameAndRestoresHeading() {
        opMode.init();
        setRobotPose(-45, 0, 90);
        opMode.gamepad1.a = true;
        opMode.loop();
        // At +90 degrees the staging point is left of the robot; clockwise restores heading.
        assertLatestPowerSigns(1, -1, 1, -1);
        assertTrue(latestPower(frontLeft) > latestPower(backLeft));

        setRobotPose(-45, 0, -90);
        clearMotors();
        opMode.gamepad1.a = false;
        opMode.loop();
        assertLatestPowerSigns(-1, 1, -1, 1);
    }

    @Test
    public void headingUsesShortestTurnAcrossWrapBoundary() {
        opMode.init();
        setRobotPose(-45, 0, 181);
        opMode.gamepad1.a = true;
        opMode.loop();
        assertLatestPowerSigns(-1, 1, -1, 1);
    }

    @Test
    public void aIsEdgeTriggeredAndBAndSticksCancelReturn() {
        startReturn();
        opMode.loop();
        assertLatestPowerSigns(-1, -1, -1, -1);
        opMode.gamepad1.b = true;
        opMode.loop();
        clearMotors();
        opMode.gamepad1.b = false;
        opMode.loop();
        assertPowers(0, 0, 0, 0);

        opMode.gamepad1.a = false;
        opMode.loop();
        opMode.gamepad1.a = true;
        opMode.loop();
        clearMotors();
        opMode.gamepad1.left_stick_y = -1;
        opMode.loop();
        assertPowers(0.45, 0.45, 0.45, 0.45);
        clearMotors();
        opMode.gamepad1.left_stick_y = 0;
        opMode.loop();
        assertPowers(0, 0, 0, 0);
    }

    @Test
    public void warningAndTimeoutLatchReturnOffUntilAnotherPress() {
        startReturn();
        status.warnOpticalTracking = true;
        opMode.loop();
        status.warnOpticalTracking = false;
        clearMotors();
        opMode.loop();
        assertPowers(0, 0, 0, 0);
        opMode.gamepad1.a = false;
        opMode.loop();
        opMode.gamepad1.a = true;
        opMode.loop();
        time = OTOSFieldNavigator.RETURN_TIMEOUT;
        opMode.loop();
        verify(opMode.telemetry).addData("Drive status",
                "Return timed out: check tracking and drive directions");
        clearMotors();
        opMode.loop();
        assertPowers(0, 0, 0, 0);
    }

    @Test
    public void faultsStopManualDrivingAndInvalidPoseStopsReturn() {
        opMode.init();
        opMode.gamepad1.left_stick_y = -1;
        status.errorLsm = true;
        opMode.loop();
        clearMotors();
        opMode.loop();
        assertPowers(0, 0, 0, 0);
        status.errorLsm = false;
        opMode.gamepad1.left_stick_y = 0;
        opMode.gamepad1.a = true;
        pose.x = Double.NaN;
        opMode.loop();
        verify(opMode.telemetry, atLeastOnce()).addData("Drive status",
                "Drive stopped: OTOS error or invalid readings");
    }

    @Test
    public void waitsForSettlingAndDoesNotResumeAfterReachingHome() {
        startReturn();
        setRobotPose(-62, 0, 0);
        opMode.loop();
        time = 0.1;
        opMode.loop();
        verify(opMode.telemetry, never()).addData("Drive status",
                "Home reached");
        time = 0.21;
        opMode.loop();
        verify(opMode.telemetry).addData("Drive status",
                "Home reached");
        pose.x = -12;
        clearMotors();
        opMode.loop();
        assertPowers(0, 0, 0, 0);
    }

    @Test
    public void savingHomeCancelsReturnAndCalibrationIsRefusedWhileMoving() {
        startReturn();
        opMode.gamepad1.options = true;
        opMode.loop();
        verify(otos, times(1)).resetTracking();
        clearMotors();
        opMode.gamepad1.options = false;
        opMode.loop();
        assertPowers(0, 0, 0, 0);
        speed.x = 4;
        opMode.gamepad1.x = true;
        opMode.loop();
        verify(otos, times(1)).calibrateImu();
        verify(opMode.telemetry).addData("Drive status",
                "Calibration refused: stop moving, then press X again");
    }

    @Test
    public void unsafeReturnIsRefusedButManualDrivingIsStillAvailable() {
        opMode.init();
        setRobotPose(0, 0, 0);
        opMode.gamepad1.a = true;
        clearMotors();
        opMode.loop();
        assertPowers(0, 0, 0, 0);
        verify(opMode.telemetry).addData("Drive status",
                "Return refused: outside mapped clear space; manually move away from obstacles/walls");
        opMode.gamepad1.left_stick_y = -1;
        clearMotors();
        opMode.loop();
        assertPowers(0.45, 0.45, 0.45, 0.45);
    }

    @Test
    public void speedChangesOncePerPressIncludingBeforeStart() {
        opMode.init();
        opMode.gamepad1.dpad_up = true;
        opMode.init_loop();
        opMode.init_loop();
        opMode.gamepad1.left_stick_y = -1;
        clearMotors();
        opMode.loop();
        assertPowers(0.5, 0.5, 0.5, 0.5);
        opMode.gamepad1.dpad_up = false;
        opMode.loop();
        opMode.gamepad1.dpad_down = true;
        opMode.loop();
        clearMotors();
        opMode.loop();
        assertPowers(0.45, 0.45, 0.45, 0.45);
        verify(opMode.telemetry, atLeastOnce()).addData("Drive power limit", "%d%%", 50);
    }

    @Test
    public void speedIsBoundedAndSimultaneousArrowsDoNotChangeIt() {
        opMode.init();
        opMode.gamepad1.dpad_up = true;
        opMode.gamepad1.dpad_down = true;
        opMode.init_loop();
        verify(opMode.telemetry, times(2)).addData("Drive power limit", "%d%%", 45);
        opMode.gamepad1.dpad_down = false;
        for (int i = 0; i < 30; i++) {
            opMode.gamepad1.dpad_up = false;
            opMode.loop();
            opMode.gamepad1.dpad_up = true;
            opMode.loop();
        }
        opMode.gamepad1.left_stick_y = -1;
        clearMotors();
        opMode.loop();
        assertPowers(1, 1, 1, 1);
        opMode.gamepad1.dpad_up = false;
        for (int i = 0; i < 30; i++) {
            opMode.gamepad1.dpad_down = false;
            opMode.loop();
            opMode.gamepad1.dpad_down = true;
            opMode.loop();
        }
        clearMotors();
        opMode.loop();
        assertPowers(0.1, 0.1, 0.1, 0.1);
    }

    @Test
    public void speedAdjustmentAlsoScalesActiveReturnWithoutCancellingIt() {
        startReturn();
        double initialPower = latestPower(frontLeft);
        opMode.gamepad1.dpad_up = true;
        clearMotors();
        opMode.loop();
        org.junit.Assert.assertEquals(initialPower * 0.5 / 0.45,
                latestPower(frontLeft), 1e-9);
        verify(opMode.telemetry, atLeastOnce()).addData("Drive status",
                "Returning: straight red-wall approach");
    }

    private void setRobotPose(double x, double y, double heading) {
        double h = Math.toRadians(heading);
        double ox = Math.cos(h) * SENSOR_BACK - Math.sin(h) * SENSOR_RIGHT;
        double oy = Math.sin(h) * SENSOR_BACK + Math.cos(h) * SENSOR_RIGHT;
        pose.x = OTOSFieldNavigator.START_X - x + ox - SENSOR_BACK;
        pose.y = -y + oy - SENSOR_RIGHT;
        pose.h = heading;
    }

    private void startReturn() {
        opMode.init();
        pose.x = -12;
        opMode.gamepad1.a = true;
        opMode.loop();
    }

    private DcMotor motor(String name) {
        DcMotor motor = mock(DcMotor.class);
        when(opMode.hardwareMap.get(DcMotor.class, name)).thenReturn(motor);
        return motor;
    }

    private void clearMotors() {
        clearInvocations(frontLeft, frontRight, backLeft, backRight);
    }

    private void assertPowers(double fl, double fr, double bl, double br) {
        verify(frontLeft, atLeastOnce()).setPower(fl);
        verify(frontRight, atLeastOnce()).setPower(fr);
        verify(backLeft, atLeastOnce()).setPower(bl);
        verify(backRight, atLeastOnce()).setPower(br);
    }

    private void assertLatestPowerSigns(int fl, int fr, int bl, int br) {
        assertPowerSign(frontLeft, fl);
        assertPowerSign(frontRight, fr);
        assertPowerSign(backLeft, bl);
        assertPowerSign(backRight, br);
    }

    private void assertPowerSign(DcMotor motor, int sign) {
        double value = latestPower(motor);
        assertTrue("Wrong drive direction: " + value, value * sign > 0);
        assertTrue("Power exceeds limit: " + value, Math.abs(value) <= 0.45);
    }

    private double latestPower(DcMotor motor) {
        org.mockito.ArgumentCaptor<Double> power =
                org.mockito.ArgumentCaptor.forClass(Double.class);
        verify(motor, atLeastOnce()).setPower(power.capture());
        return power.getValue();
    }
}
