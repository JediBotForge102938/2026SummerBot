package org.firstinspires.ftc.robotcore.external.navigation;

/** Desktop counterpart of the FTC SDK position value. */
public final class Position {
    public final DistanceUnit unit;
    public final double x, y, z;
    public final long acquisitionTime;

    public Position(DistanceUnit unit, double x, double y, double z, long acquisitionTime) {
        this.unit = unit;
        this.x = x;
        this.y = y;
        this.z = z;
        this.acquisitionTime = acquisitionTime;
    }

    public Position toUnit(DistanceUnit target) {
        return new Position(target, target.fromUnit(unit, x), target.fromUnit(unit, y),
                target.fromUnit(unit, z), acquisitionTime);
    }
}
