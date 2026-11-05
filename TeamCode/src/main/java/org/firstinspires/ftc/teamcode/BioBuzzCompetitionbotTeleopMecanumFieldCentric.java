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
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.control.DriveModePreferences;
import org.firstinspires.ftc.teamcode.control.HiveTrackingController;
import org.firstinspires.ftc.teamcode.control.MechanismCoordinator;
import org.firstinspires.ftc.teamcode.control.TeleopController;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherFeeder;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.telemetry.DriverStationTelemetry;
import org.firstinspires.ftc.teamcode.telemetry.RobotTelemetry;
import org.firstinspires.ftc.teamcode.telemetry.TelemetryPresenter;

import java.util.Arrays;

@TeleOp(name = "Mec BioBuzz CompetitionBot Field Centric Teleop", group = "CompetitionBot")
public class BioBuzzCompetitionbotTeleopMecanumFieldCentric extends OpMode {
    private final DriveSubsystem drive = new DriveSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem(false);
    private final LauncherSubsystem launcher = new LauncherSubsystem(Arrays.asList(
            new LauncherSubsystem.MotorConfig("left_launcher", DcMotor.Direction.REVERSE),
            new LauncherSubsystem.MotorConfig("right_launcher", DcMotor.Direction.FORWARD)));
    private boolean launcherInitialized;
    private boolean driveInitialized;
    private boolean intakeInitialized;
    private final VisionSubsystem vision = new VisionSubsystem();
    private TeleopController controller;
    private TelemetryPresenter display;

    @Override
    public void init() {
        drive.init(hardwareMap);
        driveInitialized = true;
        intake.init(hardwareMap);
        intakeInitialized = true;
        launcher.init(hardwareMap);
        launcherInitialized = true;
        vision.init(hardwareMap);
        DriveModePreferences preferences = new DriveModePreferences(hardwareMap.appContext);
        HiveTrackingController tracking = new HiveTrackingController(vision, preferences);
        controller = new TeleopController(drive,
                new MechanismCoordinator(intake, launcher, LauncherFeeder.DISABLED),
                preferences, tracking);
        display = new TelemetryPresenter(new DriverStationTelemetry(telemetry, hardwareMap.appContext),
                tracking, launcher, controller, intake, drive,
                output -> output.status("Competition gate", "DISABLED: positions not configured",
                        RobotTelemetry.Level.WARNING));
        controller.updateInit(gamepad1);
        display.publish();
    }

    @Override
    public void init_loop() {
        controller.updateInit(gamepad1);
        display.publish();
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
