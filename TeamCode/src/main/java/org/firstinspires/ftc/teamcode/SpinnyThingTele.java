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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import android.graphics.Color;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.BatteryChecker;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@TeleOp(name="SpinnyThingTele", group="Linear OpMode")
//@Disabled
public class SpinnyThingTele extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor LeftFront = null;
    private DcMotor LeftBack = null;
    private DcMotor RightFront = null;
    private DcMotor RightBack = null;

    private DcMotor Intake = null;
    private DcMotor Transfer = null;
    private DcMotorEx Launcher = null;
    private DcMotorEx SpindexerMotor = null;
    private Servo LaunchServo = null;


    ColorSensor IntakeSensor = null;
    ColorSensor SpinnySensor1 = null;
    ColorSensor SpinnySensor2 = null;
    ColorSensor SpinnySensor3 = null;




    private boolean Pressed = false;
    private boolean RunLauncer = false;

    private static final int EMPTY = 0;
    private static final int GREEN = 1;
    private static final int PURPLE = 2;
    private int purpleCount = 0;//should not be greater than 2
    private int greenCount = 0;//should be greater than 1

    private int[] matchMotif;
    private static final int[] motif1 = {1,2,2};
    private static final int[] motif2 = {2,1,2};
    private static final int[] motif3 = {2,2,1};


    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        LeftFront = hardwareMap.get(DcMotor.class, "LeftFront");
        LeftBack = hardwareMap.get(DcMotor.class, "LeftBack");
        RightFront = hardwareMap.get(DcMotor.class, "RightFront");
        RightBack = hardwareMap.get(DcMotor.class, "RightBack");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Transfer = hardwareMap.get(DcMotor.class, "Transfer");
        Launcher = hardwareMap.get(DcMotorEx.class, "Launcher");
        SpindexerMotor = hardwareMap.get(DcMotorEx.class, "Spindexer");
        LaunchServo = hardwareMap.get(Servo.class, "launch_servo");
        IntakeSensor = hardwareMap.get(ColorSensor.class, "intakeSensor");
        SpinnySensor1 = hardwareMap.get(ColorSensor.class, "spinnySensor1");
        SpinnySensor2 = hardwareMap.get(ColorSensor.class, "spinnySensor2");
        SpinnySensor3 = hardwareMap.get(ColorSensor.class, "spinnySensor3");

        LeftFront.setDirection(DcMotor.Direction.FORWARD);
        LeftBack.setDirection(DcMotor.Direction.REVERSE);
        RightFront.setDirection(DcMotor.Direction.FORWARD);
        RightBack.setDirection(DcMotor.Direction.REVERSE);

        Transfer.setDirection(DcMotor.Direction.FORWARD);
        Intake.setDirection(DcMotor.Direction.FORWARD);
        Launcher.setDirection(DcMotor.Direction.FORWARD);

        Launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();//

        runtime.reset();
        LaunchServo.setPosition(1);

        boolean LauncherMaxSpd = false;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            double LauncherVeloc = Launcher.getVelocity();

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   =  -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw     = -gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double frontLeftPower  = axial + lateral - yaw;
            double frontRightPower = -axial + lateral - yaw;
            double backLeftPower   = axial - lateral - yaw;
            double backRightPower  = -axial - lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower  /= max;
                frontRightPower /= max;
                backLeftPower   /= max;
                backRightPower  /= max;
            }
            LaunchServo.scaleRange(0.73, 0.83);

            Intake.setPower(-gamepad1.left_trigger);

            Transfer.setPower(gamepad1.left_trigger/1.75);

            if(gamepad1.left_bumper){
                Transfer.setPower((1/1.75));
            }

            if(gamepad1.right_bumper && !Pressed){
                RunLauncer = !RunLauncer;
                Pressed = true;
            }else if(!gamepad1.right_bumper){
                Pressed = false;
            }

            if(RunLauncer){
                Launcher.setVelocity(-3800);
            }else{
                Launcher.setVelocity(-gamepad1.right_trigger * 3800);
            }

            if(gamepad1.b && LauncherMaxSpd || gamepad1.x){
                LaunchServo.setPosition(0); //Down
            }else{

                LaunchServo.setPosition(1); // Up
            }


            telemetry.addData("Servo Pos", LaunchServo.getPosition());


            // Send calculated power to wheels
            if(!gamepad1.dpad_down){
                LeftFront.setPower(frontLeftPower / 1.5);
                RightFront.setPower(frontRightPower / 1.5);
                LeftBack.setPower(backLeftPower / 1.5);
                RightBack.setPower(backRightPower / 1.5);
            }
            else{
                LeftFront.setPower(frontLeftPower / 3);
                RightFront.setPower(frontRightPower / 3);
                LeftBack.setPower(backLeftPower / 3);
                RightBack.setPower(backRightPower / 3);
            }


            //Tolerance Value.
            LauncherMaxSpd = LauncherVeloc < -2100;

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
            telemetry.addData("Motor Speed", "%.2f", LauncherVeloc);
            telemetry.addData("Is Launcher at full speed?", LauncherMaxSpd);
            telemetry.update();



        }
    }
    ///ColorSensingStuff
    public int sensorDetectColor(ColorSensor sensor) {

        //rgb values for the color of the balls
        double[] empty = {64, 80, 99};
        double[] green = {126, 582, 240};
        double[] purple = {724, 158, 340};

        double deltaE = Math.sqrt((Math.pow(empty[0] - sensor.red(),2) + (Math.pow(empty[1] - sensor.blue(),2)) + (Math.pow(empty[2] - sensor.green(), 2))));
        double deltaR = Math.sqrt((Math.pow(green[0] - sensor.red(),2) + (Math.pow(green[1] - sensor.blue(),2)) + (Math.pow(green[2] - sensor.green(),2))));
        double deltaB = Math.sqrt((Math.pow(purple[0] - sensor.red(),2) + (Math.pow(purple[1] - sensor.blue(),2) + (Math.pow(purple[2] - sensor.green(),2)))));

        double emptyConfidence = 1 - (deltaE/3)*((1 / (deltaE + deltaR)) + (1/((deltaE + deltaB))));
        double greenConfidence = 1 - (deltaR/3)*((1 / (deltaR + deltaE)) + (1/((deltaR + deltaB))));
        double purpleConfidence = 1 - (deltaB/3)*((1 / (deltaB + deltaE)) + (1/((deltaB + deltaE))));
        double[] confidenceValues = {emptyConfidence,greenConfidence,purpleConfidence};
        int color  = findLargest(confidenceValues);

        return color;
    }

    public int findLargest(double[] array){
        double largest = array[0];
        int color = 0;

        for(int i = 1; i < array.length; i++){
            if(array[i] > largest){
                largest = array[i];//update our largest value
                color = i;//update the color we're most confident in
            }
            //if not then we don't care
        }
        return color;
    }

    ///SpinnyThingClasses


}
