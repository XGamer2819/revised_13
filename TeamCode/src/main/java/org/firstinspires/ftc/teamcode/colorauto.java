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
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;




@Autonomous(name="colorauto", group="Linear OpMode")

public class colorauto extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.

    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    private Camera cameraDetection = new Camera();

    private ColorSensor colorSensor;

    String Color_String;
    int CurrentColor;









    //init motors, servo, camera and color sensor
    private void initDcMotors() {
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "m4");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "m3");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "m1");
        rightBackDrive = hardwareMap.get(DcMotor.class, "m2");
        colorSensor = hardwareMap.get(ColorSensor.class, "Color Sensor");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);


    }

    //stop drive
    private void driveStop()
    {
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    private void drive(String direction, String color,
                       double leftFrontPower, double rightFrontPower,
                       double leftBackPower, double rightBackPower) {
        ElapsedTime runtime = new ElapsedTime();
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
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

        //drive till the color line detected.
        while (opModeIsActive()) {
            // Put loop blocks here.
            if (JavaUtil.colorToSaturation(CurrentColor) >= 0.85 && JavaUtil.colorToHue(CurrentColor) > 330 && JavaUtil.colorToHue(CurrentColor) < 360) {
                driveStop();
                colorSensor.enableLed(false);
            }
            telemetry.update();



        }
    }
    private void drive(String direction, double runtimeInseconds,
                       double leftFrontPower, double rightFrontPower,
                       double leftBackPower, double rightBackPower) {
        ElapsedTime runtime = new ElapsedTime();
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
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


    private void Post_Color_Info() {
        CurrentColor = Color.rgb(colorSensor.red(), colorSensor.green(), colorSensor.blue());
        telemetry.addData("Distance", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
        telemetry.addData("Color Hue", JavaUtil.colorToHue(CurrentColor));
        telemetry.addData("Color Saturation", JavaUtil.colorToSaturation(CurrentColor));
        telemetry.addData("Detected", Color_String);
        telemetry.update();
    }


    @Override
    public void runOpMode() throws InterruptedException {
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

            telemetry.addData("redteamprop", cameraDetection.detect());
            telemetry.update();
            leftFrontDrive.setPower(0.3);
            rightFrontDrive.setPower(0.3);
            leftBackDrive.setPower(0.3);
            rightBackDrive.setPower(0.3);
           while (opModeIsActive() && !isStopRequested()) {

               if (colorSensor.red() > 680) {
                   driveStop();
               }


               //Post_Color_Info();
                //if (JavaUtil.colorToSaturation(CurrentColor) >= 0.9 && JavaUtil.colorToHue(CurrentColor) > 330 && JavaUtil.colorToHue(CurrentColor) < 360) {
                //    driveStop();
                //    colorSensor.enableLed(false);
               telemetry.addData("Red", colorSensor.red());
               telemetry.addData("Blue", colorSensor.green());
               telemetry.addData("Green", colorSensor.blue());
               telemetry.update();

                }
                //telemetry.update();
        }
    }}
