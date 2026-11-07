package org.firstinspires.ftc.teamcode.control;

import java.util.Collections;
import java.util.List;

import org.firstinspires.ftc.teamcode.control.BioBuzzFieldMap.Point;

/** Robot-center localization and a conservative, stop-at-each-waypoint field return. */
public final class OTOSFieldNavigator {
    public static final double START_X = -BioBuzzFieldMap.FIELD_HALF_SIZE
            + BioBuzzFieldMap.ROBOT_HALF_SIZE;
    private final double sensorBack;
    private final double sensorRight;
    public static final double RETURN_TIMEOUT = 60;
    private static final Point STAGING =
            new Point(-BioBuzzFieldMap.CENTER_LIMIT + 2, 0);
    private static final double DOCK_TARGET_X = START_X + 1;
    private final BioBuzzFieldMap map = new BioBuzzFieldMap();
    private Point center = new Point(START_X, 0);
    private Point home = center;
    private double heading;
    private double homeHeading;
    private double vx;
    private double vy;
    private double turnRate;
    private List<Point> route = Collections.emptyList();
    private int waypoint;
    private boolean returning;
    private boolean wallHome = true;
    private boolean docking;
    private double started;
    private double settledSince = Double.NaN;
    private double progressTime;
    private double bestError;
    private String status = "Manual: initialize at the red wall, centered on the red hive pivot";

    public OTOSFieldNavigator(double sensorBack, double sensorRight) {
        if (!Double.isFinite(sensorBack) || !Double.isFinite(sensorRight)) {
            throw new IllegalArgumentException("OTOS offset must be finite");
        }
        this.sensorBack = sensorBack;
        this.sensorRight = sensorRight;
    }

    public static final class Command {
        public static final Command STOP = new Command(0, 0, 0);
        public final double forward;
        public final double right;
        public final double clockwise;

        private Command(double forward, double right, double clockwise) {
            this.forward = forward;
            this.right = right;
            this.clockwise = clockwise;
        }
    }

    public void updatePose(double sensorX, double sensorY, double degrees,
            double sensorVx, double sensorVy, double degreesPerSecond) {
        heading = normalize(degrees);
        turnRate = degreesPerSecond;
        double h = Math.toRadians(heading);
        double cos = Math.cos(h);
        double sin = Math.sin(h);
        double offsetX = cos * sensorBack - sin * sensorRight;
        double offsetY = sin * sensorBack + cos * sensorRight;
        // Raw sensor axes at INIT are -field X and -field Y. Remove the rotating lever arm.
        center = new Point(START_X - sensorX + offsetX - sensorBack,
                -sensorY + offsetY - sensorRight);
        double omega = Math.toRadians(turnRate);
        vx = -sensorVx - omega * offsetY;
        vy = -sensorVy + omega * offsetX;
    }

    public void saveHome() {
        cancel("Home not changed");
        if (!valid()) {
            status = "Home refused: invalid OTOS readings";
        } else if (map.isClear(center)) {
            home = center;
            homeHeading = heading;
            wallHome = false;
            status = "Home saved; field coordinates preserved";
        } else if (inDockLane()) {
            home = new Point(Math.max(DOCK_TARGET_X, center.x), 0);
            homeHeading = 0;
            wallHome = true;
            status = "Red-wall home saved; final approach stops short of wall";
        } else {
            status = "Home refused: move into mapped clear space or the red-wall starting bay";
        }
    }

    public void start(double now) {
        cancel("Planning return");
        if (!valid()) {
            status = "Return refused: invalid OTOS readings";
            return;
        }
        if (atHome() && stationary()) {
            status = "Home reached";
            return;
        }
        docking = wallHome && inDockLane();
        if (!docking && !map.isClear(center)) {
            status = "Return refused: outside mapped clear space; manually move away from obstacles/walls";
            return;
        }
        returning = true;
        started = now;
        waypoint = 0;
        route = docking ? Collections.emptyList() : map.plan(center, wallHome ? STAGING : home);
        resetProgress(now);
        status = docking ? "Returning: straight red-wall approach" : "Returning: mapped waypoints";
    }

    public Command update(double now) {
        if (!returning) {
            return Command.STOP;
        }
        if (!valid()) {
            return fail("Return stopped: invalid OTOS readings");
        }
        if (now - started >= RETURN_TIMEOUT) {
            return fail("Return timed out: check tracking and drive directions");
        }
        if (docking) {
            return dock(now);
        }
        Point target = route.get(waypoint);
        if (!map.segmentClear(center, target)) {
            return fail("Return stopped: robot drifted outside the clear route");
        }
        double distance = center.distanceTo(target);
        double headingError = normalize(homeHeading - heading);
        if (!checkProgress(now, distance + Math.abs(headingError) * 0.05)) {
            return fail("Return stopped: no progress for 4 seconds");
        }
        if (distance <= 0.5 && stationary()) {
            if (waypoint + 1 < route.size()) {
                waypoint++;
                resetProgress(now);
                return Command.STOP;
            }
            if (Math.abs(headingError) <= 3) {
                if (wallHome) {
                    docking = true;
                    resetProgress(now);
                    status = "Returning: straight red-wall approach";
                    return Command.STOP;
                }
                return finishWhenSettled(now);
            }
        }
        settledSince = Double.NaN;
        double fieldX = 0.12 * (target.x - center.x) - 0.02 * vx;
        double fieldY = 0.12 * (target.y - center.y) - 0.02 * vy;
        double magnitude = Math.hypot(fieldX, fieldY);
        if (magnitude > 0.8) {
            fieldX *= 0.8 / magnitude;
            fieldY *= 0.8 / magnitude;
        }
        // Check both the next commanded direction and measured coasting motion.
        if (!map.segmentClear(center, new Point(center.x + fieldX * 3, center.y + fieldY * 3))
                || !map.segmentClear(center, new Point(center.x + vx * 0.25, center.y + vy * 0.25))) {
            return fail("Return stopped: insufficient clearance for motion");
        }
        return robotCommand(fieldX, fieldY,
                clip(-0.025 * headingError + 0.004 * turnRate, 0.6));
    }

    private Command dock(double now) {
        if (!inDockLane()) {
            return fail("Return stopped: wall approach misaligned; manually move inward");
        }
        double targetX = Math.max(DOCK_TARGET_X, home.x);
        double distance = Math.abs(center.x - targetX);
        if (!checkProgress(now, distance)) {
            return fail("Return stopped: no progress for 4 seconds");
        }
        if (distance <= 0.5 && stationary()) {
            return finishWhenSettled(now);
        }
        settledSince = Double.NaN;
        double fieldX = clip(0.12 * (targetX - center.x) - 0.02 * vx, 0.25);
        double extent = BioBuzzFieldMap.ROBOT_HALF_SIZE
                * (Math.abs(Math.cos(Math.toRadians(heading)))
                + Math.abs(Math.sin(Math.toRadians(heading))));
        if (center.x + Math.min(fieldX * 3, vx * 0.25) - extent
                < -BioBuzzFieldMap.FIELD_HALF_SIZE) {
            return fail("Return stopped: wall clearance exhausted");
        }
        // No lateral motion or turning near the wall; do all alignment in clear space.
        return new Command(fieldX / Math.cos(Math.toRadians(heading)), 0, 0);
    }

    private boolean inDockLane() {
        return center.x >= START_X - 0.05 && center.x <= STAGING.x + 0.75
                && Math.abs(center.y) <= 0.75 && Math.abs(heading) <= 3;
    }

    private Command robotCommand(double fieldX, double fieldY, double clockwise) {
        double h = Math.toRadians(heading);
        return new Command(fieldX * Math.cos(h) + fieldY * Math.sin(h),
                fieldX * Math.sin(h) - fieldY * Math.cos(h), clockwise);
    }

    private boolean checkProgress(double now, double error) {
        if (error < bestError - 0.1) {
            bestError = error;
            progressTime = now;
        }
        return now - progressTime < 4;
    }

    private void resetProgress(double now) {
        bestError = Double.POSITIVE_INFINITY;
        progressTime = now;
        settledSince = Double.NaN;
    }

    private Command finishWhenSettled(double now) {
        if (Double.isNaN(settledSince)) {
            settledSince = now;
        } else if (now - settledSince >= 0.2) {
            cancel("Home reached");
        }
        return Command.STOP;
    }

    private Command fail(String reason) {
        cancel(reason);
        return Command.STOP;
    }

    private boolean atHome() {
        return center.distanceTo(home) <= 1 && Math.abs(normalize(homeHeading - heading)) <= 3;
    }

    private boolean stationary() {
        return Math.hypot(vx, vy) <= 1 && Math.abs(turnRate) <= 5;
    }

    private boolean valid() {
        return Double.isFinite(center.x) && Double.isFinite(center.y)
                && Double.isFinite(heading) && Double.isFinite(vx)
                && Double.isFinite(vy) && Double.isFinite(turnRate);
    }

    public void cancel(String reason) {
        returning = false;
        settledSince = Double.NaN;
        status = reason;
    }

    public boolean isReturning() {
        return returning;
    }

    public String getStatus() {
        return status;
    }

    public Point getCenter() {
        return center;
    }

    public double distanceToHome() {
        return center.distanceTo(home);
    }

    public int remainingWaypoints() {
        return returning && !docking ? route.size() - waypoint : 0;
    }

    private static double normalize(double degrees) {
        return degrees - 360 * Math.floor((degrees + 180) / 360);
    }

    private static double clip(double value, double limit) {
        return Math.max(-limit, Math.min(limit, value));
    }
}
