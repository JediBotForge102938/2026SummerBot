package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.BioBuzzLimelightClusterAdapter.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.ConfigurationSubsystem.StartingSpot;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/** Approximate BIOBUZZ locations in inches from field center; +X blue, +Y away from audience. */
public final class FieldMap {
    public static final double HALF_FIELD = 72;
    // Approximate horizontal centers: moving cells are not fixed localization landmarks.
    public static final double HIVE_X = 12;
    public static final double CELL_Y = 9.4;

    public static final class Pose {
        public final double x;
        public final double y;
        public final double headingDegrees;

        public Pose(double x, double y, double headingDegrees) {
            if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(headingDegrees)) {
                throw new IllegalArgumentException("Pose must be finite");
            }
            this.x = x;
            this.y = y;
            this.headingDegrees = headingDegrees;
        }
    }

    public static final class Region {
        public final double minX, maxX, minY, maxY;
        public Region(double minX, double maxX, double minY, double maxY) {
            this.minX = minX;
            this.maxX = maxX;
            this.minY = minY;
            this.maxY = maxY;
        }
    }

    private FieldMap() { }

    public static Pose startingPose(Alliance alliance, StartingSpot side, double lengthInches) {
        requireAlliance(alliance);
        if (side == null || !Double.isFinite(lengthInches) || lengthInches <= 0
                || lengthInches >= 2 * HALF_FIELD) {
            throw new IllegalArgumentException("Starting side and valid robot length are required");
        }
        double x = alliance == Alliance.RED ? -HIVE_X : HIVE_X;
        return new Pose(x, side == StartingSpot.AUDIENCE
                ? -HALF_FIELD + lengthInches / 2 : HALF_FIELD - lengthInches / 2,
                side == StartingSpot.AUDIENCE ? 90 : -90);
    }

    public static Pose hiveCell(Alliance alliance, StartingSpot side) {
        requireAlliance(alliance);
        if (side == null) throw new IllegalArgumentException("Starting side is required");
        return new Pose(alliance == Alliance.RED ? -HIVE_X : HIVE_X,
                side == StartingSpot.AUDIENCE ? -CELL_Y : CELL_Y, 0);
    }

    /** Approximate wall attachment points, not obstacle clearance or robot targets. */
    public static final List<Pose> FLOWER_WALL_POINTS = Collections.unmodifiableList(Arrays.asList(
            new Pose(-72, -24, 0), new Pose(72, 24, 0),
            new Pose(24, -72, 0), new Pose(-24, 72, 0)
    ));

    public static Region garden(Alliance alliance) {
        requireAlliance(alliance);
        return alliance == Alliance.RED
                ? new Region(-72, -48, -72, -70) : new Region(48, 72, 70, 72);
    }

    /** PARK is achieved by entering the alliance loading zone, not a single pose. */
    public static Region parking(Alliance alliance) {
        requireAlliance(alliance);
        return alliance == Alliance.RED
                ? new Region(-72, -61, 24, 48) : new Region(61, 72, -48, -24);
    }

    private static void requireAlliance(Alliance alliance) {
        if (alliance != Alliance.RED && alliance != Alliance.BLUE) {
            throw new IllegalArgumentException("A red or blue alliance is required");
        }
    }
}
