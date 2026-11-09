package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.ftccommon.configuration.RobotConfigFileManager;
import java.io.File;

final class ActiveRobotProfileSource {
    private ActiveRobotProfileSource() { }

    static RobotProfile load(String requiredRobot) {
        String active = new RobotConfigFileManager().getActiveConfig().getName();
        return RobotProfile.load(new File("/sdcard/FIRST/robot-profiles"), active, requiredRobot);
    }

    static boolean isSimulator() {
        return false;
    }
}
