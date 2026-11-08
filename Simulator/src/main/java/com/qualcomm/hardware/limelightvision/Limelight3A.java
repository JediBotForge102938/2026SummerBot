package com.qualcomm.hardware.limelightvision;

import com.qualcomm.robotcore.hardware.HardwareDevice;

/** Simulator-only camera: no image processing or targets. */
public final class Limelight3A implements HardwareDevice {
    private static final LLResult NO_RESULT = new LLResult();

    public void pipelineSwitch(int pipeline) {
        if (pipeline < 0 || pipeline > 9) throw new IllegalArgumentException("Invalid pipeline");
    }
    public void start() { }
    public void stop() { }
    public LLResult getLatestResult() { return NO_RESULT; }
}
