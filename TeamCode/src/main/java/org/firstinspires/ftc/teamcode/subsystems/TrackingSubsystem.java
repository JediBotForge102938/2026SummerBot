package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/** Experimental robot-center field estimate. Vision observations are not fused. */
public final class TrackingSubsystem {
    private SparkFunOTOS otos;
    private final SparkFunOTOS.Pose2D position = new SparkFunOTOS.Pose2D();
    private final SparkFunOTOS.Pose2D velocity = new SparkFunOTOS.Pose2D();
    private final SparkFunOTOS.Pose2D acceleration = new SparkFunOTOS.Pose2D();
    private FieldMap.Pose start;
    private FieldMap.Pose pose;
    private double sensorForward;
    private double sensorRight;
    private double initialX;
    private double initialY;
    private String status = "Not started";
    private boolean lost;

    public void init(HardwareMap hardware, RobotProfile profile) {
        otos = hardware.get(SparkFunOTOS.class, profile.otosName);
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);
        otos.setOffset(new SparkFunOTOS.Pose2D(0, 0, 0));
        otos.setLinearScalar(profile.otosLinearScalar);
        otos.setAngularScalar(profile.otosAngularScalar);
        if (!otos.calibrateImu()) {
            throw new IllegalStateException("OTOS IMU calibration failed; keep robot still");
        }
        otos.resetTracking();
        sensorForward = (profile.otosBackCm - profile.chassisLengthCm / 2) / 2.54;
        sensorRight = (profile.chassisWidthCm / 2 - profile.otosLeftCm) / 2.54;
    }

    public void start(FieldMap.Pose initial) {
        if (otos == null || initial == null) throw new IllegalStateException("Tracking not initialized");
        start = initial;
        lost = false;
        pose = null;
        otos.getPosVelAcc(position, velocity, acceleration);
        initialX = position.x;
        initialY = position.y;
        update(initial.headingDegrees);
    }

    public void update(double imuHeadingDegrees) {
        if (start == null) throw new IllegalStateException("Tracking not started");
        otos.getPosVelAcc(position, velocity, acceleration);
        SparkFunOTOS.Status sensorStatus = otos.getStatus();
        if (sensorStatus == null || sensorStatus.errorPaa || sensorStatus.errorLsm
                || sensorStatus.warnTiltAngle || sensorStatus.warnOpticalTracking
                || !Double.isFinite(position.x) || !Double.isFinite(position.y)
                || !Double.isFinite(position.h) || !Double.isFinite(imuHeadingDegrees)
                || !Double.isFinite(initialX) || !Double.isFinite(initialY)) {
            lost = true;
            pose = null;
            status = "Tracking lost; reinitialize at starting pose";
            return;
        }
        if (lost) return;
        double h0 = Math.toRadians(start.headingDegrees);
        double h = Math.toRadians(imuHeadingDegrees);
        double dx = position.x - initialX;
        double dy = position.y - initialY;
        double initialOffsetX = sensorForward * Math.cos(h0) + sensorRight * Math.sin(h0);
        double initialOffsetY = sensorForward * Math.sin(h0) - sensorRight * Math.cos(h0);
        double offsetX = sensorForward * Math.cos(h) + sensorRight * Math.sin(h);
        double offsetY = sensorForward * Math.sin(h) - sensorRight * Math.cos(h);
        pose = new FieldMap.Pose(start.x - dx * Math.cos(h0) + dy * Math.sin(h0)
                + initialOffsetX - offsetX,
                start.y - dx * Math.sin(h0) - dy * Math.cos(h0)
                + initialOffsetY - offsetY, normalize(imuHeadingDegrees));
        status = "Estimated; OTOS + IMU";
    }

    public FieldMap.Pose getPose() { return pose; }
    public String getStatus() { return status; }
    public double getOtosHeading() { return position.h; }

    private static double normalize(double degrees) {
        return degrees - 360 * Math.floor((degrees + 180) / 360);
    }
}
