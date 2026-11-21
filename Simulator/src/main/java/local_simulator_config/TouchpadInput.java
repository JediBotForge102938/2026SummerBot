package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.Gamepad;

public final class TouchpadInput {
    private TouchpadInput() {}

    public static boolean isTouched(Gamepad gamepad) {
        return false;
    }

    public static float x(Gamepad gamepad) {
        return 0;
    }

    public static float y(Gamepad gamepad) {
        return 0;
    }
}
