package org.firstinspires.ftc.robotcore.external.navigation;

public final class Pose3D {
    private final Position position;
    private final YawPitchRollAngles orientation;

    public Pose3D(Position position, YawPitchRollAngles orientation) {
        this.position = position;
        this.orientation = orientation;
    }

    public Position getPosition() { return position; }
    public YawPitchRollAngles getOrientation() { return orientation; }
}
