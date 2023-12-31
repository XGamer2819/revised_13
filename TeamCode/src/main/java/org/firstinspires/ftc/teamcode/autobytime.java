/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;



/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="autobytime", group="Linear OpMode")

public class autobytime extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.

    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    private Camera cameraDetection = new Camera();

    private DcMotor WheelRoller;

    static String Forward = "Forward";
    static String Backward = "Backward";
    static String Left = "Left";
    static String Right = "Right";


    //init motors, servo, camera and color sensor
    private void initDcMotors() {
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "m4");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "m3");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "m1");
        rightBackDrive = hardwareMap.get(DcMotor.class, "m2");

        WheelRoller = hardwareMap.get(DcMotor.class, "Wheel Roller");
    }

    //stop drive
    private void driveStop()
    {
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }



        //drive till the color line detected.




    private void drive(String direction, double runtimeInseconds,
                       double leftFrontPower, double rightFrontPower,
                       double leftBackPower, double rightBackPower) {
        ElapsedTime runtime = new ElapsedTime();
        leftFrontDrive.setDirection(Direction.REVERSE);
        leftBackDrive.setDirection(Direction.REVERSE);
        rightFrontDrive.setDirection(Direction.FORWARD);
        rightBackDrive.setDirection(Direction.REVERSE);
        runtime.reset();
        if (direction == "Forward") {
            leftFrontDrive.setPower(leftFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightFrontDrive.setPower(rightFrontPower);
            rightBackDrive.setPower(rightBackPower);

        }
        if (direction == "Backward") {
            leftFrontDrive.setPower(-leftFrontPower);
            leftBackDrive.setPower(-leftBackPower);
            rightFrontDrive.setPower(-rightFrontPower);
            rightBackDrive.setPower(-rightBackPower);

        }
        if (direction == "Left") {
            leftFrontDrive.setPower(-leftFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightFrontDrive.setPower(rightFrontPower);
            rightBackDrive.setPower(-rightBackPower);

        }
        if (direction == "Right") {
            leftFrontDrive.setPower(leftFrontPower);
            leftBackDrive.setPower(-leftBackPower);
            rightFrontDrive.setPower(-rightFrontPower);
            rightBackDrive.setPower(rightBackPower);
        }
        if (direction == "TurnRight") {
            leftFrontDrive.setPower(leftFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightFrontDrive.setPower(-rightFrontPower);
            rightBackDrive.setPower(-rightBackPower);
        }
        if (direction == "TurnLeft") {
            leftFrontDrive.setPower(-leftFrontPower);
            leftBackDrive.setPower(-leftBackPower);
            rightFrontDrive.setPower(rightFrontPower);
            rightBackDrive.setPower(rightBackPower);
        }

        while (opModeIsActive() && (runtime.seconds() < runtimeInseconds)) {
            telemetry.addData("Time", runtimeInseconds);
            telemetry.update();

        }
    }



    private void WheelRoller(double runtimeInseconds) {
        ElapsedTime runtime = new ElapsedTime();
        WheelRoller.setPower(-0.5);
        while (opModeIsActive() && (runtime.seconds() < runtimeInseconds)) {
            telemetry.addData("Time", runtimeInseconds);
            telemetry.update();
        }
    }
    private void Stop(double runtimeInseconds) {
        ElapsedTime runtime = new ElapsedTime();
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        while (opModeIsActive() && (runtime.seconds() < runtimeInseconds)) {
            telemetry.addData("Time", runtimeInseconds);
            telemetry.update();
        }
    }


















    @Override
    public void runOpMode() {
        initDcMotors();
        // detect team prom position
        // Camera.detect();

        //  move the robotic to the position


        // drop th e pixel on the line
        //DropPixel.Drop()

        // drive to backdrop



        //park


        // calculate how to get there



        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.





        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();


        // run until the end of the match (driver presses STOP)
        if (opModeIsActive()) {
            drive("Forward", 1.1, 0.5, 0.5, 0.5, 0.5);
            Stop(0.5);
            //WheelRoller(0.25);
            drive("Backward", 0.1, 0.5, 0.5, 0.5, 0.5);
            drive("TurnLeft", 0.875, 0.5, 0.5, 0.5, 0.5);
            Stop(0.1);
            drive("Backward", 1.0, 0.5, 0.5, 0.5, 0.5);
            Stop(0.5);
            drive("Forward", 1.0, 0, 0, 0, 0);
            drive("Left", 1.5, 0.5, 0.5, 0.5, 0.5);
            drive("Backward", 1.0, 0.5, 0.5, 0.5, 0.5);
        }
    }}
