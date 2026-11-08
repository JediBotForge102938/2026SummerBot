package local_simulator_config;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.hardware.sparkfun.SparkFunOTOSInternal;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.RobotProfile;

/** Converts simulator center odometry into StarterBot's profiled raw OTOS axes and mounting. */
public final class StarterBotOTOS extends SparkFunOTOS {
    private final SparkFunOTOSInternal sensor;
    private final double forwardCm;
    private final double rightCm;

    public StarterBotOTOS(SparkFunOTOSInternal sensor, RobotProfile profile) {
        this.sensor = sensor;
        forwardCm = profile.otosBackCm - profile.chassisLengthCm / 2;
        rightCm = profile.chassisWidthCm / 2 - profile.otosLeftCm;
    }

    @Override public void setLinearUnit(DistanceUnit unit) { sensor.setLinearUnit(unit); }
    @Override public DistanceUnit getLinearUnit() { return sensor.getLinearUnit(); }
    @Override public void setAngularUnit(AngleUnit unit) { sensor.setAngularUnit(unit); }
    @Override public AngleUnit getAngularUnit() { return sensor.getAngularUnit(); }
    @Override public boolean setLinearScalar(double scalar) { return sensor.setLinearScalar(scalar); }
    @Override public boolean setAngularScalar(double scalar) { return sensor.setAngularScalar(scalar); }
    @Override public double getLinearScalar() { return sensor.getLinearScalar(); }
    @Override public double getAngularScalar() { return sensor.getAngularScalar(); }
    @Override public boolean calibrateImu() { return sensor.calibrateImu(); }
    @Override public Status getStatus() { return sensor.getStatus(); }
    @Override public void getVersionInfo(Version hardware, Version firmware) {
        sensor.getVersionInfo(hardware, firmware);
    }
    @Override public void setOffset(Pose2D offset) {
        if (offset.x != 0 || offset.y != 0 || offset.h != 0) {
            throw new IllegalArgumentException("Simulator OTOS supports only zero hardware offset");
        }
        sensor.setOffset(offset);
    }
    @Override public void resetTracking() {
        sensor.resetTracking();
        sensor.update();
    }

    @Override public void setPosition(Pose2D position) {
        throw new UnsupportedOperationException("Simulator OTOS does not support setting position");
    }

    @Override public Pose2D getPosition() {
        Pose2D pos = new Pose2D();
        getPosVelAcc(pos, new Pose2D(), new Pose2D());
        return pos;
    }

    @Override public Pose2D getVelocity() {
        Pose2D vel = new Pose2D();
        getPosVelAcc(new Pose2D(), vel, new Pose2D());
        return vel;
    }

    @Override public Pose2D getAcceleration() {
        Pose2D acc = new Pose2D();
        getPosVelAcc(new Pose2D(), new Pose2D(), acc);
        return acc;
    }

    @Override public synchronized void getPosVelAcc(Pose2D pos, Pose2D vel, Pose2D acc) {
        Pose2D p = new Pose2D();
        Pose2D v = new Pose2D();
        Pose2D a = new Pose2D();
        sensor.getPosVelAcc(p, v, a);
        double h = AngleUnit.RADIANS.fromUnit(getAngularUnit(), p.h);
        double omega = AngleUnit.RADIANS.fromUnit(getAngularUnit(), v.h);
        double alpha = AngleUnit.RADIANS.fromUnit(getAngularUnit(), a.h);
        double f = getLinearUnit().fromCm(forwardCm);
        double r = getLinearUnit().fromCm(rightCm);
        double cos = Math.cos(h);
        double sin = Math.sin(h);
        double ox = f * cos + r * sin;
        double oy = f * sin - r * cos;
        pos.x = -p.x - ox + f;
        pos.y = -p.y - oy - r;
        pos.h = p.h;
        // CommonOdometry reports velocity and acceleration in robot axes.
        vel.x = -(v.x * cos - v.y * sin - omega * oy);
        vel.y = -(v.x * sin + v.y * cos + omega * ox);
        vel.h = v.h;
        acc.x = -(a.x * cos - a.y * sin - alpha * oy - omega * omega * ox);
        acc.y = -(a.x * sin + a.y * cos + alpha * ox - omega * omega * oy);
        acc.h = a.h;
    }
}
