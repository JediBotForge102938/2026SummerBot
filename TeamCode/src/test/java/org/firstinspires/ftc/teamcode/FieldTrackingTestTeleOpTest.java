package org.firstinspires.ftc.teamcode;

import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.control.DriveModePreferences;
import org.firstinspires.ftc.teamcode.subsystems.RobotProfile;
import org.junit.Test;
import org.mockito.MockedConstruction;
import org.mockito.MockedStatic;
import java.util.Arrays;

public class FieldTrackingTestTeleOpTest {
    @Test public void testUsesProfileAndStopsOnY() {
        RobotProfile profile = TestProfiles.starter();
        try (MockedStatic<RobotProfile> profiles = mockStatic(RobotProfile.class);
             MockedConstruction<RevHubOrientationOnRobot> orientation = mockConstruction(RevHubOrientationOnRobot.class);
             MockedConstruction<DriveModePreferences> prefs = mockConstruction(DriveModePreferences.class,
                     (mock, context) -> {
                         when(mock.getConfigurationRedAlliance()).thenReturn(true);
                         when(mock.getConfigurationFieldCentric()).thenReturn(false);
                     })) {
            profiles.when(() -> RobotProfile.loadActive(null)).thenReturn(profile);
            FieldTrackingTestTeleOp op = new FieldTrackingTestTeleOp();
            op.hardwareMap = mock(HardwareMap.class);
            op.gamepad1 = mock(Gamepad.class);
            op.telemetry = mock(Telemetry.class);
            DcMotor left = mock(DcMotor.class);
            when(op.hardwareMap.get(eq(DcMotor.class), anyString())).thenReturn(left);
            IMU imu = mock(IMU.class);
            YawPitchRollAngles angles = mock(YawPitchRollAngles.class);
            when(imu.initialize(any())).thenReturn(true);
            when(imu.getRobotYawPitchRollAngles()).thenReturn(angles);
            when(angles.getYaw(AngleUnit.DEGREES)).thenReturn(0.0);
            when(op.hardwareMap.get(IMU.class, profile.imuName)).thenReturn(imu);
            SparkFunOTOS otos = mock(SparkFunOTOS.class);
            when(otos.calibrateImu()).thenReturn(true);
            when(otos.getStatus()).thenReturn(new SparkFunOTOS.Status());
            when(op.hardwareMap.get(SparkFunOTOS.class, profile.otosName)).thenReturn(otos);
            Limelight3A camera = mock(Limelight3A.class);
            when(op.hardwareMap.get(Limelight3A.class, profile.limelightName)).thenReturn(camera);
            op.init();
            verify(otos).setLinearScalar(profile.otosLinearScalar);
            verify(camera).pipelineSwitch(profile.limelightPipeline);
            op.start();
            op.gamepad1.left_stick_y = -1;
            op.loop();
            verify(left, atLeastOnce()).setPower(0.25);
            verify(op.telemetry, atLeastOnce()).addLine(contains("Searching; not fused"));
            LLResult result = mock(LLResult.class);
            when(result.isValid()).thenReturn(true);
            LLResultTypes.FiducialResult firstTag = tag(34, -5);
            LLResultTypes.FiducialResult secondTag = tag(35, 5);
            when(result.getFiducialResults()).thenReturn(Arrays.asList(firstTag, secondTag));
            when(camera.getLatestResult()).thenReturn(result);
            op.loop();
            verify(op.telemetry).addLine(contains("<b>RED AUDIENCE</b>"));
            verify(op.telemetry).addLine(contains("SCORABLE"));
            verify(op.telemetry).addLine(contains("dX"));
            verify(op.telemetry).addLine(contains("(approx)"));
            when(result.getStaleness()).thenReturn(251L);
            op.loop();
            verify(op.telemetry, times(2)).addLine(contains("Searching; not fused"));
            op.gamepad1.y = true;
            op.loop();
            verify(left, atLeastOnce()).setPower(0);
            op.stop();
            verify(camera).stop();
        }
    }

    private static LLResultTypes.FiducialResult tag(int id, double x) {
        LLResultTypes.FiducialResult tag = mock(LLResultTypes.FiducialResult.class);
        when(tag.getFiducialId()).thenReturn(id);
        when(tag.getTargetPoseCameraSpace()).thenReturn(new Pose3D(
                new Position(DistanceUnit.INCH, x, 0, 60, 0),
                new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, -10, 0)));
        return tag;
    }
}
