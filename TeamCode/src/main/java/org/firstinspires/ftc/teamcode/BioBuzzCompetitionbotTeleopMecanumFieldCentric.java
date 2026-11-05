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

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;

@TeleOp(name = "Mec BioBuzz CompetitionBot Field Centric Teleop", group = "CompetitionBot")
public class BioBuzzCompetitionbotTeleopMecanumFieldCentric extends OpMode {
    private final DriveSubsystem drive = new DriveSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final LauncherSubsystem launcher = new LauncherSubsystem();

    @Override
    public void init() {
        drive.init(hardwareMap);
        intake.init(hardwareMap);
        launcher.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Press A to reset field heading");
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            drive.resetHeading();
        }

        drive.driveFieldRelative(-gamepad1.left_stick_y,
                gamepad1.left_stick_x, gamepad1.right_stick_x);
        double intakePower = gamepad1.right_trigger - gamepad1.left_trigger;
        if (launcher.update(gamepad1.right_bumper)) {
            intakePower += 0.5;
        }
        intake.setPower(intakePower);

        telemetry.addData("Motors", "FL (%.2f), FR (%.2f), BL (%.2f), BR (%.2f)",
                drive.getLeftFrontPower(), drive.getRightFrontPower(),
                drive.getLeftBackPower(), drive.getRightBackPower());
        telemetry.addData("Heading", "%.1f degrees", drive.getHeadingDegrees());
        telemetry.addData("Triggers", "left (%.2f), right (%.2f)",
                gamepad1.left_trigger, gamepad1.right_trigger);
    }
}
