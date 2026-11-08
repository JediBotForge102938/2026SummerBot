package com.qualcomm.hardware.limelightvision;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public final class LLResultTypes {
    private LLResultTypes() { }

    public static final class FiducialResult {
        public int getFiducialId() { throw new UnsupportedOperationException("No simulated fiducials"); }
        public Pose3D getTargetPoseCameraSpace() { throw new UnsupportedOperationException("No simulated fiducials"); }
        public double getTargetXDegrees() { throw new UnsupportedOperationException("No simulated fiducials"); }
        public double getTargetYDegrees() { throw new UnsupportedOperationException("No simulated fiducials"); }
    }
}
