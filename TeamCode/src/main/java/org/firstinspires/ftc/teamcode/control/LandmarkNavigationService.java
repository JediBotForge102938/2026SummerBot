package org.firstinspires.ftc.teamcode.control;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.localization.Localizer;
import com.pedropathing.paths.PathConstraints;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FieldMap;
import org.firstinspires.ftc.teamcode.subsystems.RobotProfile;
import org.firstinspires.ftc.teamcode.subsystems.TrackingSubsystem;

import java.util.Collections;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/** Reusable, guarded Pedro autonomous follower using the established OTOS + hub-IMU pose. */
public final class LandmarkNavigationService {
    private final RobotProfile profile;
    private final TrackingSubsystem tracking;
    private final LandmarkFieldMap map;
    private final Follower follower;
    private List<LandmarkFieldMap.Waypoint> route = Collections.emptyList();
    private LandmarkFieldMap.Destination target;
    private int waypointIndex;
    private boolean active;
    private boolean awaitingSettle;
    private double startedAt;
    private double progressAt;
    private double bestError;
    private double lastUpdateAt;
    private double powerLimit;
    private double settledAt = Double.NaN;
    private String phase = "Idle";
    private String status = "Ready";

    public LandmarkNavigationService(DriveSubsystem drive,
            TrackingSubsystem tracking, RobotProfile profile) {
        if (drive == null || tracking == null || profile == null) {
            throw new IllegalArgumentException("Navigation requires drive, tracking and profile");
        }
        this.profile = profile;
        this.tracking = tracking;
        map = new LandmarkFieldMap(profile);
        Localizer localizer = new PedroTrackingLocalizer(tracking);
        PedroDriveOutput output = new PedroDriveOutput(drive);
        FollowerConstants constants = new FollowerConstants();
        constants.defaults();
        RobotProfile.NavigationSettings settings = profile.navigation;
        constants.setCoefficientsTranslationalPIDF(
                new PIDFCoefficients(settings.translationalP, 0,
                        settings.translationalD, 0));
        constants.setCoefficientsHeadingPIDF(
                new PIDFCoefficients(settings.headingP, 0, settings.headingD, 0));
        constants.setCoefficientsDrivePIDF(
                new FilteredPIDFCoefficients(settings.driveP, 0,
                        settings.driveD, 0, 0));
        constants.setAutomaticHoldEnd(true);
        output.setXVelocity(settings.forwardVelocity);
        output.setYVelocity(settings.lateralVelocity);
        PathConstraints constraints = PathConstraints.defaultConstraints.copy();
        constraints.setTranslationalConstraint(settings.arrivalToleranceIn);
        constraints.setHeadingConstraint(Math.toRadians(settings.headingToleranceDegrees));
        constraints.setVelocityConstraint(settings.stationarySpeedInPerSec);
        follower = new Follower(constants, localizer, output, constraints);
        follower.setMaxPowerScaling(settings.transitPower);
    }

    public boolean start(LandmarkFieldMap.Destination destination, double nowSeconds) {
        cancel("Planning");
        if (!profile.isAutonomousReady()) {
            status = "Autonomous blocked: physical geometry and Pedro tuning are not verified";
            phase = "Blocked";
            return false;
        }
        FieldMap.Pose pose = tracking.getPose();
        if (!tracking.hasValidMotion() || pose == null) {
            status = "Autonomous refused: tracked pose unavailable";
            phase = "Failed";
            return false;
        }
        if (destination == null || !Double.isFinite(nowSeconds)) {
            status = "Autonomous refused: missing target or invalid clock";
            phase = "Failed";
            return false;
        }
        try {
            route = map.plan(pose, destination);
        } catch (IllegalArgumentException | IllegalStateException exception) {
            status = safeMessage(exception);
            phase = "Failed";
            return false;
        }
        target = destination;
        waypointIndex = 0;
        startedAt = nowSeconds;
        lastUpdateAt = nowSeconds;
        active = true;
        awaitingSettle = route.isEmpty();
        phase = route.isEmpty() ? "Settling" : route.get(0).segment.name();
        status = route.isEmpty() ? "Checking destination arrival" : "Following " + phase.toLowerCase(Locale.ROOT);
        resetProgress(nowSeconds);
        if (!route.isEmpty()) beginLeg(pose);
        else stopDrive();
        return true;
    }

    public void update(double nowSeconds) {
        if (!active) return;
        FieldMap.Pose pose = tracking.getPose();
        if (!tracking.hasValidMotion() || pose == null) {
            fail("Navigation stopped: pose/motion unavailable or OTOS warning/error");
            return;
        }
        if (!Double.isFinite(nowSeconds) || nowSeconds < lastUpdateAt
                || nowSeconds - startedAt >= profile.navigation.timeoutSeconds) {
            fail("Navigation timed out");
            return;
        }
        lastUpdateAt = nowSeconds;
        if (route.isEmpty()) {
            if (nowSeconds - progressAt >= profile.navigation.progressTimeoutSeconds) {
                fail("Navigation stopped: no stationary arrival within progress timeout");
                return;
            }
            settleAtTarget(nowSeconds, pose);
            return;
        }

        LandmarkFieldMap.Waypoint waypoint = route.get(waypointIndex);
        if (waypoint.segment == LandmarkFieldMap.Segment.FINAL_APPROACH
                && Math.abs(angleDifference(pose.headingDegrees,
                target.robotPose.headingDegrees)) > profile.navigation.headingToleranceDegrees) {
            fail("Navigation stopped: final wall approach is misaligned");
            return;
        }
        boolean clear = waypoint.segment == LandmarkFieldMap.Segment.TRANSIT
                ? map.isTransitClear(pose)
                : map.segmentStillClear(pose, waypoint, target);
        if (!clear) {
            fail(String.format(Locale.US,
                    "Navigation stopped: footprint left %s route at X %.1f Y %.1f",
                    waypoint.segment.name().toLowerCase(Locale.ROOT), pose.x, pose.y));
            return;
        }
        double error = (waypoint.segment == LandmarkFieldMap.Segment.TRANSIT
                ? follower.getTotalDistanceRemaining() : distance(pose, waypoint.pose))
                + Math.abs(angleDifference(pose.headingDegrees, waypoint.pose.headingDegrees)) * 0.05;
        if (error < bestError - profile.navigation.progressThresholdIn) {
            bestError = error;
            progressAt = nowSeconds;
        } else if (nowSeconds - progressAt >= profile.navigation.progressTimeoutSeconds) {
            fail(String.format(Locale.US, "Navigation stopped: no meaningful progress for %.1f"
                    + " seconds (error %.2f in)", profile.navigation.progressTimeoutSeconds,
                    error));
            return;
        }

        if (withinArrival(pose, waypoint.pose)) {
            if (!awaitingSettle) {
                follower.holdPoint(pedroPose(waypoint.pose));
                awaitingSettle = true;
            }
            powerLimit = Math.min(powerFor(waypoint.segment), profile.navigation.finalApproachPower);
            follower.setMaxPowerScaling(powerLimit);
            follower.update();
            if (!tracking.isStationary(profile.navigation.stationarySpeedInPerSec,
                    profile.navigation.stationaryTurnRateDegreesPerSec)) {
                settledAt = Double.NaN;
            } else if (Double.isNaN(settledAt)) {
                beginSettle(pose, nowSeconds);
            } else if (nowSeconds - settledAt >= profile.navigation.settleSeconds) {
                if (waypointIndex + 1 < route.size()) {
                    waypointIndex++;
                    awaitingSettle = false;
                    settledAt = Double.NaN;
                    resetProgress(nowSeconds);
                    beginLeg(pose);
                } else {
                    complete();
                }
            }
            return;
        }
        if (awaitingSettle) {
            settledAt = Double.NaN;
        }
        double targetDistance = distance(pose, waypoint.pose);
        powerLimit = targetDistance <= profile.navigation.precisionDistanceIn
                ? Math.min(powerFor(waypoint.segment), profile.navigation.finalApproachPower)
                : powerFor(waypoint.segment);
        follower.setMaxPowerScaling(powerLimit);
        follower.update();
        status = "Following " + waypoint.segment.name().toLowerCase(Locale.ROOT)
                + " to " + target.name;
    }

    public void cancel(String reason) {
        active = false;
        awaitingSettle = false;
        settledAt = Double.NaN;
        powerLimit = 0;
        if (follower != null) {
            follower.breakFollowing();
        }
        phase = "Idle";
        status = reason == null ? "Cancelled" : reason;
    }

    public boolean isActive() { return active; }
    public String getStatus() { return status; }
    public String getPhase() { return phase; }
    public LandmarkFieldMap.Destination getTarget() { return target; }
    public int getRemainingWaypoints() {
        return active ? route.size() - waypointIndex : 0;
    }
    public double getPowerLimit() {
        if (!active || route.isEmpty()) return 0;
        return powerLimit;
    }

    private void beginLeg(FieldMap.Pose current) {
        LandmarkFieldMap.Waypoint waypoint = route.get(waypointIndex);
        Pose from = pedroPose(current);
        if (waypoint.segment == LandmarkFieldMap.Segment.TRANSIT) {
            PathBuilder builder = follower.pathBuilder();
            int end = waypointIndex;
            while (end + 1 < route.size()
                    && route.get(end + 1).segment == LandmarkFieldMap.Segment.TRANSIT) end++;
            for (int index = waypointIndex; index <= end; index++) {
                LandmarkFieldMap.Waypoint part = route.get(index);
                Pose to = pedroPose(part.pose);
                if (part.curveControlPoints.isEmpty()) {
                    builder.addPath(new BezierLine(from, to));
                } else {
                    ArrayList<Pose> controls = new ArrayList<>();
                    for (FieldMap.Pose control : part.curveControlPoints) {
                        controls.add(pedroPose(control));
                    }
                    builder.addPath(new BezierCurve(controls));
                }
                from = to;
            }
            builder.setGlobalLinearHeadingInterpolation(
                    Math.toRadians(current.headingDegrees), from.getHeading());
            follower.followPath(builder.build(), true);
            waypointIndex = end;
        } else {
            Pose to = pedroPose(waypoint.pose);
            Path path = new Path(new BezierLine(from, to));
            path.setConstantHeadingInterpolation(to.getHeading());
            follower.followPath(path, true);
        }
        powerLimit = powerFor(waypoint.segment);
        follower.setMaxPowerScaling(powerLimit);
        phase = waypoint.segment.name();
    }

    private double powerFor(LandmarkFieldMap.Segment segment) {
        return segment == LandmarkFieldMap.Segment.TRANSIT
                || segment == LandmarkFieldMap.Segment.DEPARTURE
                ? profile.navigation.transitPower : profile.navigation.finalApproachPower;
    }

    private void settleAtTarget(double nowSeconds, FieldMap.Pose pose) {
        if (!withinArrival(pose, target.robotPose)) {
            fail("Navigation stopped: target is not within arrival tolerance");
            return;
        }
        stopDrive();
        if (!tracking.isStationary(profile.navigation.stationarySpeedInPerSec,
                profile.navigation.stationaryTurnRateDegreesPerSec)) {
            settledAt = Double.NaN;
        } else if (Double.isNaN(settledAt)) {
            beginSettle(pose, nowSeconds);
        } else if (nowSeconds - settledAt >= profile.navigation.settleSeconds) {
            complete();
        }
    }

    private boolean withinArrival(FieldMap.Pose pose, FieldMap.Pose goal) {
        return distance(pose, goal) <= profile.navigation.arrivalToleranceIn
                && Math.abs(angleDifference(pose.headingDegrees, goal.headingDegrees))
                        <= profile.navigation.headingToleranceDegrees;
    }

    private void resetProgress(double nowSeconds) {
        progressAt = nowSeconds;
        bestError = Double.POSITIVE_INFINITY;
    }

    private void beginSettle(FieldMap.Pose pose, double nowSeconds) {
        awaitingSettle = true;
        settledAt = nowSeconds;
        phase = "Settling";
        status = "Settling at " + target.name;
    }

    private void complete() {
        active = false;
        powerLimit = 0;
        follower.breakFollowing();
        phase = "Arrived";
        status = "Arrived at " + target.name;
        settledAt = Double.NaN;
    }

    private void fail(String reason) {
        active = false;
        powerLimit = 0;
        follower.breakFollowing();
        phase = "Failed";
        status = reason;
        settledAt = Double.NaN;
    }

    private void stopDrive() {
        powerLimit = 0;
        follower.breakFollowing();
    }

    private static Pose pedroPose(FieldMap.Pose pose) {
        return new Pose(pose.x, pose.y, Math.toRadians(pose.headingDegrees));
    }

    private static String safeMessage(RuntimeException exception) {
        String message = exception.getMessage();
        return message == null || message.trim().isEmpty()
                ? "Navigation refused: route planning failed" : message;
    }

    private static double distance(FieldMap.Pose a, FieldMap.Pose b) {
        return Math.hypot(a.x - b.x, a.y - b.y);
    }

    private static double angleDifference(double a, double b) {
        return a - 360 * Math.floor((a - b + 180) / 360) - b;
    }
}
