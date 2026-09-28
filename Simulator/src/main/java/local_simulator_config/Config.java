package virtual_robot.config;

import com.qualcomm.robotcore.hardware.configuration.MotorType;
import javafx.scene.image.Image;
import virtual_robot.controller.Game;
import virtual_robot.controller.VirtualBot;
import virtual_robot.games.BioBuzz;
import virtual_robot.robots.classes.BioBuzzStarterBotSim;

/**
 * Root-project override of virtual_robot's Config. The file is intentionally outside the standard
 * package directory so the source-set exclusion removes only the submodule's original Config.java.
 */
public class Config {
    public static final double X_MIN_FRACTION = 0;
    public static final double X_MAX_FRACTION = 1;
    public static final double Y_MIN_FRACTION = 0;
    public static final double Y_MAX_FRACTION = 1;

    public static final double FIELD_WIDTH = 648;

    /**
     * Use physical USB or Bluetooth gamepads instead of the simulator's on-screen gamepad.
     */
    public static final boolean USE_VIRTUAL_GAMEPAD = false;

    public static final Image BACKGROUND = new Image("/virtual_robot/assets/biobuzz648.bmp");

    public static final boolean HOLD_CONTROLS_BY_DEFAULT = false;

    public static final Game GAME = new BioBuzz();

    public static final double FIELD_FRICTION_COEFF = 10;

    public static final MotorType DEFAULT_DRIVE_MOTOR_TYPE = MotorType.Gobilda192;

    public static final Class<? extends VirtualBot> DEFAULT_BOT = BioBuzzStarterBotSim.class;
}
