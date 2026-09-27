/*
 * Copyright (c) 2025 FIRST
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this list
 * of conditions and the following disclaimer in the documentation and/or other materials
 * provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior written
 * permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*
 * This file includes a teleop (driver-controlled) file for the Mecanum Drive goBILDA® StarterBot Base
 * Chassis/Intake for the 2026-2027 FIRST® Tech Challenge. It leverages a mecanum drive system for
 * robot mobility, one motor driving an intake roller, and two servos which pull elements out of corners.
 */

@TeleOp(name = "Stryker: Mecanum Chassis Teleop", group = "StarterBot")
//@Disabled
public class StarterBotMecChassisTeleop extends OpMode {

    // Declare OpMode members.
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor intake = null;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;

    // Set up a variable for each drive wheel to save power level for telemetry.
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    // Create a variable to set to the intake.
    double intakePower;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step.
         */
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        intake = getOptional(hardwareMap, DcMotor.class, "intake");
        leftIntakeServo = getOptional(hardwareMap, CRServo.class, "left_intake_servo");
        rightIntakeServo = getOptional(hardwareMap, CRServo.class, "right_intake_servo");

        /*
         * To drive forward, most robots need the motor on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
         * MUST make robot go forward. So adjust these two lines based on your first test drive.
         * Note: The settings here assume direct drive on left and right wheels. Gear
         * Reduction or 90 Deg drives may require direction flips
         */
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        if (intake != null) {
            intake.setZeroPowerBehavior(BRAKE);
        }

        /*
         * set Feeders to an initial value to initialize the servo controller
         */
        if (leftIntakeServo != null) {
            leftIntakeServo.setPower(0);
        }
        if (rightIntakeServo != null) {
            rightIntakeServo.setPower(0);
        }

        /*
         * Much like our drivetrain motors, we set the right intake servo to reverse so that both
         * servos work to pull elements into the intake.
         */
        if (rightIntakeServo != null) {
            rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        /*
         * Here we call a function called mecanumDrive. The mecanumDrive function takes the input from
         * the joysticks, and applies power to the drive motors to move the robot as requested by the driver. Note,
         * moving the left joystick forward/back moves all motors forwards/back, moving the right joystick left/right
         * rotates the robot clockwise or counterclockwise, and moving the left joystick left moves the motors in the
         * right way to create a sideways "strafe" movement. Combinations of these inputs can be used to create
         * more complex maneuvers.
         *
         * Apply the standard simulator joystick convention while preserving the real robot's
         * motor orientation convention.
         */
        mecanumDrive(-gamepad1.left_stick_y,
                gamepad1.left_stick_x, gamepad1.right_stick_x);

        /*
         * Set the intake power variable to equal the right trigger, minus the left trigger.
         * Each trigger outputs a signal from 0-1, with 0 as fully released, and 1 fully depressed.
         * This gives us proportional control of the intake speed. The speed increases as we pull
         * the right trigger further. It's occasionally helpful to be able to reverse the intake,
         * so we also factor in the  left trigger. If the left trigger is fully depressed, the intakePower variable
         * will be -1. If the right trigger is fully depressed, the variable will be 1. If the driver pulls both
         * triggers, the intake will remain off.
         * We use this technique (creating a variable, and setting it to our control inputs) to allow us to avoid
         * setting the same motors/servos power more than once per loop. That can create erratic behavior.
         */
        intakePower = gamepad1.right_trigger - gamepad1.left_trigger;

        if (intake != null) {
            intake.setPower(intakePower);
        }

        if (leftIntakeServo != null) {
            leftIntakeServo.setPower(intakePower);
        }
        if (rightIntakeServo != null) {
            rightIntakeServo.setPower(intakePower);
        }

        /*
         * Show motor powers on the Driver Station via telemetry.
         */
        telemetry.addData("Motors", "FL (%.2f), FR (%.2f), BL(%.2f), BR(%.2f)",
                leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
        telemetry.addData("Motor commands", "port 2 FL (%.2f), port 3 FR (%.2f), port 0 BL (%.2f), port 1 BR (%.2f)",
                leftFrontDrive.getPower(), rightFrontDrive.getPower(),
                leftBackDrive.getPower(), rightBackDrive.getPower());
        telemetry.addData("Triggers", "left (%.2f, right (%.2f)",gamepad1.left_trigger, gamepad1.right_trigger);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    void mecanumDrive(double forward, double strafe, double rotate){

        /* the denominator is the largest motor power (absolute value) or 1
         * This ensures all the powers maintain the same ratio,
         * but only if at least one is out of the range [-1, 1]
         */
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        leftFrontPower = (forward + strafe + rotate) / denominator;
        rightFrontPower = (forward - strafe - rotate) / denominator;
        leftBackPower = (forward - strafe + rotate) / denominator;
        rightBackPower = (forward + strafe - rotate) / denominator;

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

    }

    private static <T> T getOptional(
            HardwareMap hardwareMap, Class<? extends T> deviceType, String deviceName) {
        try {
            return hardwareMap.get(deviceType, deviceName);
        } catch (IllegalArgumentException ignored) {
            return null;
        }
    }
}
