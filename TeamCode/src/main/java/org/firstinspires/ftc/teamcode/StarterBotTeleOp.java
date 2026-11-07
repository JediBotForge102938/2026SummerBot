/*   MIT License
 *   Copyright (c) [2026] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:
 *
 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.
 *
 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.control.DriveModePreferences;
import org.firstinspires.ftc.teamcode.control.HiveTrackingController;
import org.firstinspires.ftc.teamcode.control.MechanismCoordinator;
import org.firstinspires.ftc.teamcode.control.TeleopController;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ConfigurationSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WindmillSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotProfile;
import org.firstinspires.ftc.teamcode.telemetry.StarterBotTelemetry;

@TeleOp(name = "StarterBot")
public class StarterBotTeleOp extends OpMode {
    private final DriveSubsystem drive = new DriveSubsystem();
    private final ConfigurationSubsystem configuration = new ConfigurationSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem(true);
    private final LauncherSubsystem launcher = new LauncherSubsystem();
    private final WindmillSubsystem windmill = new WindmillSubsystem();
    private final VisionSubsystem vision = new VisionSubsystem();
    private TeleopController controller;
    private StarterBotTelemetry display;

    @Override
    public void init() {
        RobotProfile profile = RobotProfile.loadActive("starterbot");
        drive.init(hardwareMap, profile);
        intake.init(hardwareMap);
        launcher.init(hardwareMap);
        windmill.init(hardwareMap);
        vision.init(hardwareMap, profile);
        DriveModePreferences preferences = new DriveModePreferences(hardwareMap.appContext);
        HiveTrackingController tracking = new HiveTrackingController(
                vision, preferences, drive::getYawRateDegreesPerSecond);
        controller = new TeleopController(drive, new MechanismCoordinator(intake, launcher, windmill),
                preferences, tracking);
        display = new StarterBotTelemetry(telemetry, configuration, controller, tracking, launcher);
        configuration.init(telemetry, gamepad1, preferences);
        configuration.setRobotProfileName(profile.name);
        publishConfiguration();
    }

    @Override
    public void init_loop() {
        configuration.updateInit(gamepad1);
        publishConfiguration();
    }

    @Override
    public void start() {
        if (!configuration.isLocked()) {
            configuration.lock();
            drive.resetHeading(configuration.getStartingHeadingDegrees(),
                    configuration.getFieldForwardHeadingDegrees());
            controller.configure(configuration, gamepad1);
        }
        display.publish();
    }

    private void publishConfiguration() {
        telemetry.clearAll();
        configuration.publish();
        telemetry.update();
    }

    @Override
    public void loop() {
        controller.update(gamepad1);
        display.publish();
    }

    @Override
    public void stop() {
        gamepad1.stopRumble();
        if (controller != null) {
            controller.stop();
        }
        vision.stop();
        if (display != null) {
            display.publish();
        }
    }
}
