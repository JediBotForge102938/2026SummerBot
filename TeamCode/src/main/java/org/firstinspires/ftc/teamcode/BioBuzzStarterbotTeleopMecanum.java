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
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WindmillSubsystem;
import org.firstinspires.ftc.teamcode.telemetry.DriverStationTelemetry;
import org.firstinspires.ftc.teamcode.telemetry.TelemetryPresenter;

@TeleOp(name = "Mec BioBuzz StarterBot Teleop", group = "StarterBot")
public class BioBuzzStarterbotTeleopMecanum extends OpMode {
    private final DriveSubsystem drive = new DriveSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem(true);
    private final LauncherSubsystem launcher = new LauncherSubsystem();
    private final WindmillSubsystem windmill = new WindmillSubsystem();
    private final VisionSubsystem vision = new VisionSubsystem();
    private TeleopController controller;
    private TelemetryPresenter display;

    @Override
    public void init() {
        drive.init(hardwareMap);
        intake.init(hardwareMap);
        launcher.init(hardwareMap);
        windmill.init(hardwareMap);
        vision.init(hardwareMap);
        DriveModePreferences preferences = new DriveModePreferences(hardwareMap.appContext);
        HiveTrackingController tracking = new HiveTrackingController(vision, preferences);
        controller = new TeleopController(drive, new MechanismCoordinator(intake, launcher, windmill),
                preferences, tracking);
        display = new TelemetryPresenter(new DriverStationTelemetry(telemetry, hardwareMap.appContext),
                tracking, launcher, controller, intake, drive);
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
        }
        vision.stop();
        if (display != null) {
            display.publish();
        }
    }
}
