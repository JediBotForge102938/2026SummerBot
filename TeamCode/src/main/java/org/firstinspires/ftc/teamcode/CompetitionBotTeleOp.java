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
import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherFeeder;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotProfile;
import org.firstinspires.ftc.teamcode.telemetry.StarterBotTelemetry;

@TeleOp(name = "CompetitionBot")
public class CompetitionBotTeleOp extends OpMode {
    private final DriveSubsystem drive = new DriveSubsystem();
    private final ConfigurationSubsystem configuration = new ConfigurationSubsystem();
    private IntakeSubsystem intake;
    private LauncherSubsystem launcher;
    private GateSubsystem gate;
    private boolean gateInitialized;
    private boolean gateOpen;
    private boolean gateButtonWasDown;
    private boolean gateTwoControllers;
    private boolean launcherInitialized;
    private boolean driveInitialized;
    private boolean intakeInitialized;
    private final VisionSubsystem vision = new VisionSubsystem();
    private TeleopController controller;
    private StarterBotTelemetry display;

    @Override
    public void init() {
        RobotProfile profile = RobotProfile.loadActive("competitionbot");
        gate = new GateSubsystem(profile.launcherGate);
        gate.init(hardwareMap);
        gateInitialized = true;
        gateOpen = false;
        gateButtonWasDown = gamepad1.b;
        drive.init(hardwareMap, profile);
        driveInitialized = true;
        intake = new IntakeSubsystem(false, profile);
        intake.init(hardwareMap);
        intakeInitialized = true;
        launcher = new LauncherSubsystem(profile.launcherMotors, profile.launcherPidf,
                profile.launcherFeedVelocityMargin);
        launcher.init(hardwareMap);
        launcherInitialized = true;
        vision.init(hardwareMap, profile);
        DriveModePreferences preferences = new DriveModePreferences(hardwareMap.appContext);
        HiveTrackingController tracking = new HiveTrackingController(
                vision, preferences, drive::getYawRateDegreesPerSecond);
        controller = new TeleopController(drive,
                new MechanismCoordinator(intake, launcher, LauncherFeeder.DISABLED,
                        profile.intakeFeedAssistPercent),
                preferences, tracking);
        controller.setIntakeProfile(profile.robot, profile.intakeDefaultSpeedPercent);
        display = new StarterBotTelemetry(telemetry, configuration, controller, tracking, launcher);
        configuration.init(telemetry, gamepad1, preferences, true);
        gateTwoControllers = configuration.isTwoControllers();
        gateButtonWasDown = gateGamepad().b;
        configuration.setRobotProfileName(profile.name);
        publishConfiguration();
    }

    @Override
    public void init_loop() {
        configuration.updateInit(gamepad1, gamepad2);
        updateGateToggle();
        publishConfiguration();
    }

    @Override
    public void start() {
        if (!configuration.isLocked()) {
            configuration.lock();
            drive.resetHeading(configuration.getStartingHeadingDegrees(),
                    configuration.getFieldForwardHeadingDegrees());
            if (configuration.isTwoControllers()) {
                controller.configure(configuration, gamepad1, gamepad2);
            } else {
                controller.configure(configuration, gamepad1);
            }
            gateTwoControllers = configuration.isTwoControllers();
            gateButtonWasDown = gateGamepad().b;
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
        updateGateToggle();
        if (configuration.isTwoControllers()) {
            controller.update(gamepad1, gamepad2);
        } else {
            controller.update(gamepad1);
        }
        display.publish();
    }

    private void updateGateToggle() {
        boolean selectedTwo = configuration.isTwoControllers();
        boolean buttonDown = (selectedTwo ? gamepad2 : gamepad1).b;
        if (selectedTwo != gateTwoControllers) {
            gateTwoControllers = selectedTwo;
            gateButtonWasDown = buttonDown;
        }
        if (buttonDown && !gateButtonWasDown) {
            gateOpen = !gateOpen;
            gate.update(gateOpen);
        }
        gateButtonWasDown = buttonDown;
    }

    private com.qualcomm.robotcore.hardware.Gamepad gateGamepad() {
        return gateTwoControllers ? gamepad2 : gamepad1;
    }

    @Override
    public void stop() {
        gamepad1.stopRumble();
        if (configuration.isTwoControllers()) {
            gamepad2.stopRumble();
        }
        if (gateInitialized) {
            gate.update(false);
            gateOpen = false;
        }
        if (controller != null) {
            controller.stop();
        } else {
            if (launcherInitialized) {
                launcher.update(false);
            }
            if (intakeInitialized) {
                intake.setPower(0);
            }
            if (driveInitialized) {
                drive.driveRobotRelative(0, 0, 0);
            }
        }
        vision.stop();
        if (display != null) {
            display.publish();
        }
    }
}
