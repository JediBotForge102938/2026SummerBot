package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.teamcode.BioBuzzLimelightClusterAdapter.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.ConfigurationSubsystem.StartingSpot;
import org.firstinspires.ftc.teamcode.subsystems.FieldMap;
import org.firstinspires.ftc.teamcode.subsystems.RobotProfile;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/** Navigation-only targets and footprint-aware routes; field objects remain separate from poses. */
public final class LandmarkFieldMap {
    private static final double APPROACH_ENTRY_DRIFT_MARGIN_IN = 1.0;
    public enum Kind { FLOWER, GARDEN, PARKING, SHOOTING_START, HIVE_SHOOTING, SAVED_HOME }
    public enum Segment { DEPARTURE, TRANSIT, APPROACH_CORRIDOR, FINAL_APPROACH }

    public static final class Destination {
        public final String name;
        public final Kind kind;
        public final FieldMap.Pose robotPose;
        public final FieldMap.Pose objectPose;
        public final FieldMap.Region region;
        private final boolean reverseFinalApproach;
        private final boolean edgeParking;

        private Destination(String name, Kind kind, FieldMap.Pose robotPose,
                FieldMap.Pose objectPose, FieldMap.Region region,
                boolean reverseFinalApproach) {
            this(name, kind, robotPose, objectPose, region, reverseFinalApproach, false);
        }

        private Destination(String name, Kind kind, FieldMap.Pose robotPose,
                FieldMap.Pose objectPose, FieldMap.Region region,
                boolean reverseFinalApproach, boolean edgeParking) {
            this.name = name;
            this.kind = kind;
            this.robotPose = robotPose;
            this.objectPose = objectPose;
            this.region = region;
            this.reverseFinalApproach = reverseFinalApproach;
            this.edgeParking = edgeParking;
        }

        public static Destination savedHome(FieldMap.Pose pose) {
            if (pose == null) throw new IllegalArgumentException("Saved home pose is required");
            return new Destination("Saved home", Kind.SAVED_HOME, pose, null, null, false);
        }
    }

    public static final class Waypoint {
        public final FieldMap.Pose pose;
        public final Segment segment;
        public final List<FieldMap.Pose> curveControlPoints;
        private final double departureWallClearance;

        private Waypoint(FieldMap.Pose pose, Segment segment) {
            this(pose, segment, Collections.emptyList(), 0);
        }

        private Waypoint(FieldMap.Pose pose, Segment segment, List<FieldMap.Pose> controls) {
            this(pose, segment, controls, 0);
        }

        private Waypoint(FieldMap.Pose pose, Segment segment, List<FieldMap.Pose> controls,
                double departureWallClearance) {
            this.pose = pose;
            this.segment = segment;
            curveControlPoints = Collections.unmodifiableList(new ArrayList<>(controls));
            this.departureWallClearance = departureWallClearance;
        }
    }

    private final RobotProfile profile;
    private final double halfLength;
    private final double halfWidth;
    private final double robotRadius;
    private final double keepOutX;
    private final double keepOutY;
    private final double transitLimit;

    public LandmarkFieldMap(RobotProfile profile) {
        if (profile == null) throw new IllegalArgumentException("Robot profile is required");
        if (profile.navigation == null) {
            throw new IllegalStateException("Landmark travel requires a profile navigation section");
        }
        this.profile = profile;
        halfLength = profile.chassisLengthCm / 2.0 / 2.54;
        halfWidth = profile.chassisWidthCm / 2.0 / 2.54;
        robotRadius = Math.hypot(halfLength, halfWidth);
        keepOutX = BioBuzzFieldMap.FRAME_HALF_WIDTH + robotRadius
                + profile.navigation.transitClearanceIn;
        keepOutY = BioBuzzFieldMap.FRAME_HALF_DEPTH + robotRadius
                + profile.navigation.transitClearanceIn;
        transitLimit = BioBuzzFieldMap.FIELD_HALF_SIZE - BioBuzzFieldMap.PERIMETER_STRIP
                - robotRadius - profile.navigation.wallClearanceIn;
    }

    /** Returns the same twelve destinations for either alliance selection. */
    public List<Destination> destinations() {
        List<Destination> result = new ArrayList<>();
        for (FieldMap.Pose flower : FieldMap.FLOWER_WALL_POINTS) {
            FieldMap.Pose target;
            if (Math.abs(flower.x) == FieldMap.HALF_FIELD) {
                double heading = flower.x < 0 ? 180 : 0;
                double x = flower.x - Math.cos(Math.toRadians(heading))
                        * (halfLength + profile.navigation.flowerStandoffIn);
                target = new FieldMap.Pose(x, flower.y, heading);
            } else {
                double heading = flower.y < 0 ? -90 : 90;
                double y = flower.y - Math.sin(Math.toRadians(heading))
                        * (halfLength + profile.navigation.flowerStandoffIn);
                target = new FieldMap.Pose(flower.x, y, heading);
            }
            String[] flowerNames = {"Red wall flower", "Blue wall flower",
                    "Red audience flower", "Blue scoring flower"};
            result.add(new Destination(flowerNames[result.size()], Kind.FLOWER,
                    target, flower, null, false));
        }
        result.add(gardenCollectionDestination(Alliance.RED, "Red garden"));
        result.add(gardenCollectionDestination(Alliance.BLUE, "Blue garden"));
        result.add(parkingDestination(Alliance.RED));
        result.add(parkingDestination(Alliance.BLUE));
        for (Alliance alliance : Arrays.asList(Alliance.RED, Alliance.BLUE)) {
            for (StartingSpot side : Arrays.asList(StartingSpot.AUDIENCE, StartingSpot.SCORING)) {
                FieldMap.Pose start = FieldMap.startingPose(alliance, side,
                        profile.chassisLengthCm / 2.54);
                String color = alliance == Alliance.RED ? "Red" : "Blue";
                String startSide = side == StartingSpot.AUDIENCE ? "audience" : "scoring";
                result.add(new Destination(color + " " + startSide + " shooting start",
                        Kind.SHOOTING_START, start, FieldMap.hiveCell(alliance, side),
                        null, true));
            }
        }
        return Collections.unmodifiableList(result);
    }

    public Destination parkingDestination(Alliance alliance) {
        FieldMap.Region region = FieldMap.parking(alliance);
        boolean red = alliance == Alliance.RED;
        double x = 72 - halfLength - profile.navigation.flowerStandoffIn;
        return new Destination(red ? "Red parking" : "Blue parking", Kind.PARKING,
                new FieldMap.Pose(red ? -x : x, (region.minY + region.maxY) / 2,
                        red ? 180 : 0), null, region, false);
    }

    public Destination gardenDestination(Alliance alliance) {
        FieldMap.Region garden = FieldMap.garden(alliance);
        for (Destination destination : destinations()) {
            if (destination.kind == Kind.GARDEN
                    && destination.region.minX == garden.minX) {
                return destination;
            }
        }
        throw new IllegalStateException("Alliance garden is not mapped");
    }

    /** Faces the front intake toward the garden's audience/scoring-wall strip. */
    public Destination gardenCollectionDestination(Alliance alliance) {
        return gardenCollectionDestination(alliance,
                alliance == Alliance.RED ? "Red garden collection" : "Blue garden collection");
    }

    private Destination gardenCollectionDestination(Alliance alliance, String name) {
        FieldMap.Region garden = FieldMap.garden(alliance);
        boolean red = alliance == Alliance.RED;
        double edgeY = red ? garden.maxY : garden.minY;
        double y = edgeY + (red ? 1 : -1)
                * (halfLength + profile.navigation.flowerStandoffIn);
        double x = (red ? -1 : 1)
                * (FieldMap.HALF_FIELD - halfWidth - profile.navigation.wallClearanceIn);
        FieldMap.Pose pose = new FieldMap.Pose(x,
                y, red ? -90 : 90);
        return new Destination(name, Kind.GARDEN, pose,
                new FieldMap.Pose(pose.x, edgeY, 0), garden, false);
    }

    /** Leaves turning clearance and points the rear launcher toward the selected cell. */
    public Destination hiveShootingDestination(Alliance alliance, StartingSpot side) {
        FieldMap.Pose cell = FieldMap.hiveCell(alliance, side);
        double y = transitLimit - 0.5;
        FieldMap.Pose pose = new FieldMap.Pose(cell.x,
                side == StartingSpot.AUDIENCE ? -y : y,
                side == StartingSpot.AUDIENCE ? -90 : 90);
        if (!isTransitClear(pose)) {
            throw new IllegalStateException("No clear hive shooting position for this profile");
        }
        return new Destination(alliance + " " + side + " shooting approach",
                Kind.HIVE_SHOOTING, pose, cell, null, false);
    }

    public Destination parkingDestination(Alliance alliance, StartingSpot startingSpot) {
        if (startingSpot == null) throw new IllegalArgumentException("Starting spot is required");
        Destination center = parkingDestination(alliance);
        double overlap = profile.navigation.parkingOverlapIn
                + (startingSpot == StartingSpot.SCORING ? 1.0 : 0.0);
        if ((alliance == Alliance.BLUE && startingSpot == StartingSpot.AUDIENCE)
                || (alliance == Alliance.RED && startingSpot == StartingSpot.SCORING)) {
            overlap += 1.0;
        }
        if (overlap <= profile.navigation.arrivalToleranceIn
                || overlap >= (center.region.maxY - center.region.minY) / 2) {
            throw new IllegalStateException("Parking overlap must exceed arrival tolerance and leave the middle free");
        }
        double y = startingSpot == StartingSpot.AUDIENCE
                ? center.region.minY - halfWidth + overlap
                : center.region.maxY + halfWidth - overlap;
        Destination edge = new Destination(center.name + (startingSpot == StartingSpot.AUDIENCE
                ? " audience edge" : " scoring edge"), Kind.PARKING,
                new FieldMap.Pose(center.robotPose.x, y, alliance == Alliance.RED ? 0 : 180),
                null, center.region, true, true);
        return edge;
    }

    public boolean isTransitClear(FieldMap.Pose pose) {
        if (pose == null || !finite(pose.x, pose.y, pose.headingDegrees)
                || Math.abs(pose.x) > transitLimit || Math.abs(pose.y) > transitLimit) {
            return false;
        }
        double hx = Math.abs(pose.x);
        double hy = Math.abs(pose.y);
        return hx > keepOutX || hy > keepOutY;
    }

    public boolean isSafeHome(FieldMap.Pose pose) {
        return isFootprintClear(pose, profile.navigation.wallClearanceIn);
    }

    public List<Waypoint> plan(FieldMap.Pose start, Destination destination) {
        if (start == null || destination == null) {
            throw new IllegalArgumentException("Start pose and destination are required");
        }
        FieldMap.Pose goal = destination.robotPose;
        if (!finite(start.x, start.y, start.headingDegrees)) {
            throw new IllegalArgumentException("Tracked pose is invalid");
        }
        if (!isFootprintClear(start, departureClearance(start))
                || !isFootprintClear(goal, destination.kind == Kind.SHOOTING_START
                ? 0 : profile.navigation.wallClearanceIn)) {
            throw new IllegalStateException("Route refused: start or target footprint is unsafe");
        }
        if (distance(start, goal) <= profile.navigation.arrivalToleranceIn
                && Math.abs(angleDifference(start.headingDegrees, goal.headingDegrees))
                <= profile.navigation.headingToleranceDegrees) {
            return Collections.emptyList();
        }

        List<Waypoint> plan = new ArrayList<>();
        FieldMap.Pose routeStart = start;
        if (!isTransitClear(start)) {
            List<FieldMap.Pose> departure = safeDeparture(start);
            if (departure.isEmpty()) {
                throw new IllegalStateException(
                        "Route refused: start is outside clear space and has no safe straight departure");
            }
            for (FieldMap.Pose point : departure) {
                plan.add(new Waypoint(point, Segment.DEPARTURE, Collections.emptyList(),
                        departureClearance(start)));
            }
            routeStart = departure.get(departure.size() - 1);
        }

        Approach approach = findPreApproach(goal, destination);
        List<FieldMap.Pose> transit = transitRoute(routeStart, approach.entry, destination);
        plan.addAll(smoothTransit(routeStart, transit, goal.headingDegrees, destination));
        if (distance(approach.entry, approach.point) > 0.1) {
            plan.add(new Waypoint(approach.point, Segment.APPROACH_CORRIDOR));
        }
        if (!segmentFootprintClear(approach.point, goal, goal.headingDegrees,
                destination.kind == Kind.SHOOTING_START ? 0
                        : profile.navigation.wallClearanceIn)) {
            throw new IllegalStateException("Route refused: final approach violates clearance");
        }
        plan.add(new Waypoint(goal, Segment.FINAL_APPROACH));
        if (destination.edgeParking) {
            FieldMap.Pose previous = start;
            for (Waypoint waypoint : plan) {
                if (!parkingMiddleClear(previous, waypoint.pose, destination,
                        waypoint.segment == Segment.TRANSIT ? robotRadius : halfLength,
                        waypoint.segment == Segment.TRANSIT ? robotRadius : halfWidth)) {
                    throw new IllegalStateException("Route refused: parking route crosses partner space");
                }
                previous = waypoint.pose;
            }
        }
        return Collections.unmodifiableList(plan);
    }

    public boolean segmentStillClear(FieldMap.Pose from, FieldMap.Pose to, Segment segment,
            Destination destination) {
        if (!parkingMiddleClear(from, to, destination,
                segment == Segment.TRANSIT ? robotRadius : halfLength,
                segment == Segment.TRANSIT ? robotRadius : halfWidth)) return false;
        if (segment == Segment.TRANSIT) {
            return segmentTransitClear(from, to);
        }

        if (segment == Segment.DEPARTURE) {
            return segmentFootprintClear(from, to, from.headingDegrees,
                    departureClearance(from));
        }
        return isFootprintClear(from, destination.kind == Kind.SHOOTING_START ? 0
                : profile.navigation.wallClearanceIn)
                && Math.abs(angleDifference(from.headingDegrees, to.headingDegrees))
                <= profile.navigation.headingToleranceDegrees
                && segmentInterpolatedFootprintClear(from, to,
                destination.kind == Kind.SHOOTING_START ? 0
                        : profile.navigation.wallClearanceIn);
    }

    public boolean segmentStillClear(FieldMap.Pose from, Waypoint waypoint,
            Destination destination) {
        if (destination.edgeParking && !parkingMiddleClear(from, waypoint.pose, destination,
                waypoint.segment == Segment.TRANSIT ? robotRadius : halfLength,
                waypoint.segment == Segment.TRANSIT ? robotRadius : halfWidth)) return false;
        if (waypoint.segment == Segment.DEPARTURE) {
            return Math.abs(angleDifference(from.headingDegrees, waypoint.pose.headingDegrees))
                    <= profile.navigation.headingToleranceDegrees
                    && segmentFootprintClear(from, waypoint.pose, from.headingDegrees,
                    waypoint.departureWallClearance);
        }
        return segmentStillClear(from, waypoint.pose, waypoint.segment, destination);
    }

    private double departureClearance(FieldMap.Pose start) {
        for (Alliance alliance : new Alliance[] {Alliance.RED, Alliance.BLUE}) {
            FieldMap.Pose garden = gardenCollectionDestination(alliance).robotPose;
            double wallLimit = FieldMap.HALF_FIELD - halfLength;
            if (Math.abs(start.x - garden.x) <= profile.navigation.arrivalToleranceIn + 0.5
                    && Math.signum(start.y) == Math.signum(garden.y)
                    && Math.abs(start.y) >= Math.abs(garden.y)
                    && Math.abs(start.y) <= wallLimit + profile.navigation.startWallPlacementToleranceIn
                    && Math.abs(angleDifference(start.headingDegrees, garden.headingDegrees))
                    <= profile.navigation.headingToleranceDegrees) {
                return -profile.navigation.startWallPlacementToleranceIn;
            }
        }
        for (Destination destination : destinations()) {
            if (destination.kind == Kind.SHOOTING_START
                    && distance(start, destination.robotPose)
                    <= profile.navigation.startWallPlacementToleranceIn
                    && Math.abs(angleDifference(start.headingDegrees,
                    destination.robotPose.headingDegrees))
                    <= profile.navigation.headingToleranceDegrees) {
                return -profile.navigation.startWallPlacementToleranceIn;
            }
        }
        return profile.navigation.wallClearanceIn;
    }

    private List<FieldMap.Pose> safeDeparture(FieldMap.Pose start) {
        double angle = Math.toRadians(start.headingDegrees);
        double clearance = departureClearance(start);
        double limit = transitLimit - profile.navigation.arrivalToleranceIn
                - APPROACH_ENTRY_DRIFT_MARGIN_IN;
        for (double distance = 0.5; distance <= 72; distance += 0.5) {
            for (int direction : new int[] {1, -1}) {
                FieldMap.Pose retreat = new FieldMap.Pose(
                        start.x + direction * Math.cos(angle) * distance,
                        start.y + direction * Math.sin(angle) * distance, start.headingDegrees);
                if (!segmentFootprintClear(start, retreat, start.headingDegrees, clearance)) continue;
                if (isTransitClear(retreat) && Math.abs(retreat.x) <= limit
                        && Math.abs(retreat.y) <= limit) return Collections.singletonList(retreat);
                FieldMap.Pose entry = new FieldMap.Pose(
                        Math.max(-limit, Math.min(limit, retreat.x)),
                        Math.max(-limit, Math.min(limit, retreat.y)), start.headingDegrees);
                if (isTransitClear(entry) && segmentFootprintClear(retreat, entry,
                        start.headingDegrees, clearance)) {
                    return Arrays.asList(retreat, entry);
                }
            }
        }
        return Collections.emptyList();
    }

    private List<Waypoint> smoothTransit(FieldMap.Pose start, List<FieldMap.Pose> points,
            double heading, Destination destination) {
        List<Waypoint> result = new ArrayList<>();
        FieldMap.Pose previous = start;
        for (int i = 0; i < points.size(); i++) {
            FieldMap.Pose corner = withHeading(points.get(i), heading);
            if (i + 1 < points.size()) {
                FieldMap.Pose next = points.get(i + 1);
                double cut = Math.min(profile.navigation.curveInsetIn,
                        Math.min(distance(previous, corner), distance(corner, next)) / 3);
                boolean rounded = false;
                while (cut >= 0.05) {
                    FieldMap.Pose entry = toward(corner, previous, cut, heading);
                    FieldMap.Pose exit = toward(corner, next, cut, heading);
                    List<FieldMap.Pose> controls = Arrays.asList(entry, corner, exit);
                    if (segmentTransitClear(previous, entry) && isCurveTransitClear(controls)
                            && parkingMiddleClear(previous, entry, destination, robotRadius, robotRadius)
                            && parkingCurveClear(controls, destination)) {
                        result.add(new Waypoint(entry, Segment.TRANSIT));
                        result.add(new Waypoint(exit, Segment.TRANSIT, controls));
                        previous = exit;
                        rounded = true;
                        break;
                    }
                    cut /= 2;
                }
                if (rounded) continue;
            }
            result.add(new Waypoint(corner, Segment.TRANSIT));
            previous = corner;
        }
        return result;
    }

    /** A Bezier stays inside its control-point hull, so this certifies every t, not samples. */
    public boolean isCurveTransitClear(List<FieldMap.Pose> controls) {
        if (controls == null || controls.size() < 3) return false;
        double minX = Double.POSITIVE_INFINITY, maxX = Double.NEGATIVE_INFINITY;
        double minY = Double.POSITIVE_INFINITY, maxY = Double.NEGATIVE_INFINITY;
        for (FieldMap.Pose point : controls) {
            if (!isTransitClear(point)) return false;
            minX = Math.min(minX, point.x);
            maxX = Math.max(maxX, point.x);
            minY = Math.min(minY, point.y);
            maxY = Math.max(maxY, point.y);
        }
        return minX > keepOutX || maxX < -keepOutX
                || minY > keepOutY || maxY < -keepOutY;
    }

    private static FieldMap.Pose withHeading(FieldMap.Pose point, double heading) {
        return new FieldMap.Pose(point.x, point.y, heading);
    }

    private static FieldMap.Pose toward(FieldMap.Pose from, FieldMap.Pose to,
            double distance, double heading) {
        double scale = distance / distance(from, to);
        return new FieldMap.Pose(from.x + (to.x - from.x) * scale,
                from.y + (to.y - from.y) * scale, heading);
    }

    private Approach findPreApproach(FieldMap.Pose goal, Destination destination) {
        double angle = Math.toRadians(goal.headingDegrees);
        double direction = destination.reverseFinalApproach ? 1 : -1;
        double wallClearance = destination.kind == Kind.SHOOTING_START
                ? 0 : profile.navigation.wallClearanceIn;
        FieldMap.Pose point = null;
        for (double distance = 8; distance <= 72; distance += 0.5) {
            FieldMap.Pose candidate = new FieldMap.Pose(goal.x
                    + direction * Math.cos(angle) * distance,
                    goal.y + direction * Math.sin(angle) * distance, goal.headingDegrees);
            if (isFootprintClear(candidate, wallClearance)
                    && segmentFootprintClear(candidate, goal, goal.headingDegrees, wallClearance)) {
                point = candidate;
                break;
            }
        }
        if (point == null) {
            throw new IllegalStateException("Route refused: no clear heading-aligned approach");
        }
        FieldMap.Pose entry = point;
        // Leave room for arrival tolerance and tracking drift before the transit safety boundary.
        double entryLimit = transitLimit - profile.navigation.arrivalToleranceIn
                - APPROACH_ENTRY_DRIFT_MARGIN_IN;
        if (!isTransitClear(entry) || Math.abs(entry.x) > entryLimit
                || Math.abs(entry.y) > entryLimit
                || !parkingMiddleClear(entry, entry, destination, robotRadius, robotRadius)) {
            double stepX = Math.abs(point.x) > entryLimit || destination.edgeParking
                    ? -Math.signum(point.x) : 0;
            double stepY = Math.abs(point.y) > entryLimit ? -Math.signum(point.y) : 0;
            for (double offset = 0.5; offset <= 72; offset += 0.5) {
                FieldMap.Pose candidate = new FieldMap.Pose(
                        point.x + stepX * offset,
                        point.y + stepY * offset, goal.headingDegrees);
                if (isTransitClear(candidate) && Math.abs(candidate.x) <= entryLimit
                        && Math.abs(candidate.y) <= entryLimit
                        && parkingMiddleClear(candidate, candidate, destination,
                        robotRadius + profile.navigation.arrivalToleranceIn,
                        robotRadius + profile.navigation.arrivalToleranceIn)
                        && segmentFootprintClear(candidate, point, goal.headingDegrees,
                        wallClearance)) {
                    entry = candidate;
                    break;
                }
            }
        }
        if (!isTransitClear(entry)
                || !parkingMiddleClear(entry, entry, destination, robotRadius, robotRadius)
                || !segmentFootprintClear(entry, point, goal.headingDegrees, wallClearance)) {
            throw new IllegalStateException("Route refused: no safe entry to final approach corridor");
        }
        return new Approach(entry, point);
    }

    private static final class Approach {
        final FieldMap.Pose entry;
        final FieldMap.Pose point;
        Approach(FieldMap.Pose entry, FieldMap.Pose point) {
            this.entry = entry;
            this.point = point;
        }
    }

    private List<FieldMap.Pose> transitRoute(FieldMap.Pose start, FieldMap.Pose goal,
            Destination destination) {
        if (!isTransitClear(start) || !isTransitClear(goal)) {
            throw new IllegalStateException("Route refused: transit endpoint is not clear");
        }
        double sideLimit = destination.edgeParking
                ? Math.min(transitLimit, Math.min(Math.abs(destination.region.minX),
                        Math.abs(destination.region.maxX)) - robotRadius) : transitLimit;
        double xOffset = (sideLimit - keepOutX) / 2;
        double yOffset = (transitLimit - keepOutY) / 2;
        if (xOffset <= 0 || yOffset <= 0) {
            throw new IllegalStateException("Route refused: no clearance around field frame");
        }
        List<FieldMap.Pose> nodes = Arrays.asList(start, goal,
                new FieldMap.Pose(-keepOutX - xOffset, -keepOutY - yOffset,
                        start.headingDegrees),
                new FieldMap.Pose(-keepOutX - xOffset, keepOutY + yOffset,
                        start.headingDegrees),
                new FieldMap.Pose(keepOutX + xOffset, -keepOutY - yOffset,
                        start.headingDegrees),
                new FieldMap.Pose(keepOutX + xOffset, keepOutY + yOffset,
                        start.headingDegrees));
        double[] distances = new double[nodes.size()];
        Arrays.fill(distances, Double.POSITIVE_INFINITY);
        distances[0] = 0;
        int[] previous = new int[nodes.size()];
        Arrays.fill(previous, -1);
        boolean[] visited = new boolean[nodes.size()];
        for (int step = 0; step < nodes.size(); step++) {
            int current = -1;
            for (int i = 0; i < nodes.size(); i++) {
                if (!visited[i] && (current < 0 || distances[i] < distances[current])) {
                    current = i;
                }
            }
            if (current < 0 || !Double.isFinite(distances[current])) break;
            visited[current] = true;
            for (int next = 0; next < nodes.size(); next++) {
                if (!visited[next] && segmentTransitClear(nodes.get(current), nodes.get(next))
                        && parkingMiddleClear(nodes.get(current), nodes.get(next),
                        destination, robotRadius, robotRadius)) {
                    double candidate = distances[current] + distance(nodes.get(current), nodes.get(next));
                    if (candidate < distances[next]) {
                        distances[next] = candidate;
                        previous[next] = current;
                    }
                }
            }
        }
        if (!Double.isFinite(distances[1])) {
            throw new IllegalStateException("Route refused: no footprint-clear path around field frame");
        }
        List<FieldMap.Pose> route = new ArrayList<>();
        for (int index = 1; index != 0; index = previous[index]) route.add(nodes.get(index));
        Collections.reverse(route);
        return route;
    }

    private boolean segmentTransitClear(FieldMap.Pose start, FieldMap.Pose end) {
        if (!isTransitClear(start) || !isTransitClear(end)) return false;
        return !segmentIntersectsRectangle(start.x, start.y, end.x, end.y, keepOutX, keepOutY);
    }

    private boolean parkingMiddleClear(FieldMap.Pose start, FieldMap.Pose end,
            Destination destination, double extentX, double extentY) {
        if (!destination.edgeParking) return true;
        FieldMap.Region region = destination.region;
        double centerX = (region.minX + region.maxX) / 2;
        double centerY = (region.minY + region.maxY) / 2;
        return !segmentIntersectsRectangle(start.x - centerX, start.y - centerY,
                end.x - centerX, end.y - centerY,
                (region.maxX - region.minX) / 2 + extentX,
                (region.maxY - region.minY) / 4 + extentY);
    }

    private boolean parkingCurveClear(List<FieldMap.Pose> controls, Destination destination) {
        if (!destination.edgeParking) return true;
        double minX = Double.POSITIVE_INFINITY, maxX = Double.NEGATIVE_INFINITY;
        double minY = Double.POSITIVE_INFINITY, maxY = Double.NEGATIVE_INFINITY;
        for (FieldMap.Pose point : controls) {
            minX = Math.min(minX, point.x);
            maxX = Math.max(maxX, point.x);
            minY = Math.min(minY, point.y);
            maxY = Math.max(maxY, point.y);
        }
        FieldMap.Region r = destination.region;
        return maxX < r.minX - robotRadius || minX > r.maxX + robotRadius
                || maxY < r.minY + (r.maxY - r.minY) / 4 - robotRadius
                || minY > r.maxY - (r.maxY - r.minY) / 4 + robotRadius;
    }

    private boolean segmentFootprintClear(FieldMap.Pose start, FieldMap.Pose end,
            double heading, double minWallClearance) {
        return segmentInterpolatedFootprintClear(withHeading(start, heading),
                withHeading(end, heading), minWallClearance);
    }

    private boolean segmentInterpolatedFootprintClear(FieldMap.Pose start, FieldMap.Pose end,
            double minWallClearance) {
        double length = distance(start, end);
        int samples = Math.max(1, (int) Math.ceil(length / 0.5));
        for (int i = 0; i <= samples; i++) {
            double t = (double) i / samples;
            FieldMap.Pose sample = new FieldMap.Pose(
                    start.x + (end.x - start.x) * t,
                    start.y + (end.y - start.y) * t, start.headingDegrees
                            + angleDifference(end.headingDegrees, start.headingDegrees) * t);
            if (!isFootprintClear(sample, minWallClearance)) return false;
        }
        return true;
    }

    private boolean isFootprintClear(FieldMap.Pose pose, double wallClearance) {
        if (pose == null || !finite(pose.x, pose.y, pose.headingDegrees)) return false;
        double angle = Math.toRadians(pose.headingDegrees);
        double extentX = Math.abs(Math.cos(angle)) * halfLength
                + Math.abs(Math.sin(angle)) * halfWidth;
        double extentY = Math.abs(Math.sin(angle)) * halfLength
                + Math.abs(Math.cos(angle)) * halfWidth;
        if (Math.abs(pose.x) + extentX + wallClearance > 72 + 1e-6
                || Math.abs(pose.y) + extentY + wallClearance > 72 + 1e-6) return false;
        return Math.abs(pose.x) > BioBuzzFieldMap.FRAME_HALF_WIDTH
                        + extentX + profile.navigation.transitClearanceIn
                || Math.abs(pose.y) > BioBuzzFieldMap.FRAME_HALF_DEPTH
                        + extentY + profile.navigation.transitClearanceIn;
    }

    private boolean segmentIntersectsRectangle(double x1, double y1, double x2, double y2,
            double halfX, double halfY) {
        double low = 0;
        double high = 1;
        double dx = x2 - x1;
        double dy = y2 - y1;
        double[] p = {-dx, dx, -dy, dy};
        double[] q = {x1 + halfX, halfX - x1, y1 + halfY, halfY - y1};
        for (int i = 0; i < p.length; i++) {
            if (Math.abs(p[i]) < 1e-9) {
                if (q[i] < 0) return false;
            } else {
                double t = q[i] / p[i];
                if (p[i] < 0) low = Math.max(low, t);
                else high = Math.min(high, t);
                if (low > high) return false;
            }
        }
        return true;
    }

    private static double distance(FieldMap.Pose a, FieldMap.Pose b) {
        return Math.hypot(a.x - b.x, a.y - b.y);
    }

    private static double angleDifference(double a, double b) {
        return normalize(a - b);
    }

    private static double normalize(double degrees) {
        return degrees - 360 * Math.floor((degrees + 180) / 360);
    }

    private static boolean finite(double... values) {
        for (double value : values) if (!Double.isFinite(value)) return false;
        return true;
    }
}
