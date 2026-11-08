package com.qualcomm.hardware.limelightvision;

import java.util.Collections;
import java.util.List;

/** No camera targets are simulated. */
public final class LLResult {
    public boolean isValid() { return false; }
    public long getStaleness() { return 0; }
    public double getCaptureLatency() { return 0; }
    public double getTargetingLatency() { return 0; }
    public List<LLResultTypes.FiducialResult> getFiducialResults() { return Collections.emptyList(); }
}
