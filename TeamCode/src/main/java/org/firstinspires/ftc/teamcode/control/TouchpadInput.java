package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.Gamepad;

public final class TouchpadInput {
    private TouchpadInput() {}

    public static boolean isTouched(Gamepad gamepad) {
        return gamepad.touchpad_finger_1;
    }

    public static float x(Gamepad gamepad) {
        return gamepad.touchpad_finger_1_x;
    }

    public static float y(Gamepad gamepad) {
        return gamepad.touchpad_finger_1_y;
    }
}
