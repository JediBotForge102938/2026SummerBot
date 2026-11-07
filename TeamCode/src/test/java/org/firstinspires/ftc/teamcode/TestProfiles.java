package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.subsystems.RobotProfile;

import java.io.File;

public final class TestProfiles {
    private TestProfiles() { }

    public static RobotProfile starter() {
        return RobotProfile.load(new File("../robot-configs"), "biobuzz-starterbot", "starterbot");
    }
}
