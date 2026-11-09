package org.firstinspires.ftc.teamcode.control;

/** Rising-edge menu/action handling for the landmark travel experiment. */
public final class LandmarkTravelControls {
    public static final class Input {
        public boolean a;
        public boolean b;
        public boolean x;
        public boolean y;
        public boolean options;
        public boolean start;
        public boolean dpadUp;
        public boolean dpadDown;
        public double leftX;
        public double leftY;
        public double rightX;
    }

    public static final class Actions {
        public final boolean stop;
        public final boolean cancel;
        public final boolean toggleRoute;
        public final boolean manual;
        public final boolean calibrate;
        public final boolean saveHome;
        public final int selectionDelta;

        private Actions(boolean stop, boolean cancel, boolean toggleRoute,
                boolean manual, boolean calibrate, boolean saveHome, int selectionDelta) {
            this.stop = stop;
            this.cancel = cancel;
            this.toggleRoute = toggleRoute;
            this.manual = manual;
            this.calibrate = calibrate;
            this.saveHome = saveHome;
            this.selectionDelta = selectionDelta;
        }
    }

    private static final double DEAD_BAND = 0.08;
    private boolean previousA;
    private boolean previousX;
    private boolean previousOptions;
    private boolean previousStart;
    private boolean previousUp;
    private boolean previousDown;

    /** Samples controls at START so held INIT buttons cannot become fresh actions. */
    public void seed(Input input) {
        previousA = input.a;
        previousX = input.x;
        previousOptions = input.options;
        previousStart = input.start;
        previousUp = input.dpadUp;
        previousDown = input.dpadDown;
    }

    public Actions update(Input input, boolean routeActive) {
        boolean options = input.options || input.start;
        boolean saveEdge = options && !previousOptions && !previousStart;
        boolean calibrateEdge = input.x && !previousX;
        boolean routeEdge = input.a && !previousA;
        int selectionDelta = input.dpadUp && !previousUp && !input.dpadDown ? -1
                : input.dpadDown && !previousDown && !input.dpadUp ? 1 : 0;
        boolean manual = Math.abs(input.leftX) > DEAD_BAND
                || Math.abs(input.leftY) > DEAD_BAND
                || Math.abs(input.rightX) > DEAD_BAND;
        previousA = input.a;
        previousX = input.x;
        previousOptions = options;
        previousStart = options;
        previousUp = input.dpadUp;
        previousDown = input.dpadDown;

        if (input.y) return new Actions(true, true, false, false, false, false, 0);
        if (input.b) return new Actions(true, true, false, false, false, false, 0);
        if (calibrateEdge) return new Actions(true, true, false, false, true, false, 0);
        if (saveEdge) return new Actions(true, true, false, false, false, true, 0);
        if (manual) return new Actions(false, true, false, true, false, false, 0);
        return new Actions(false, false, routeEdge,
                false, false, false, selectionDelta);
    }
}
