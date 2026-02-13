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

import static org.firstinspires.ftc.teamcode.TransferedWayPoint.findLargest;

import static java.lang.Math.abs;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.BatteryChecker;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.Arrays;
import java.util.List;
import java.util.Objects;

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

@TeleOp(name="DRIVE", group="Linear OpMode")
//@Disabled
public class CurrentOmni extends LinearOpMode {

    // Declare Opmode member for the Limelight 3A camera
    private Limelight3A limelight;

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor LeftFront = null;
    private DcMotor LeftBack = null;
    private DcMotor RightFront = null;
    private DcMotor RightBack = null;

    private DcMotor Index = null;
    private DcMotorEx Launcher = null;
    private Servo Pusher = null;
    private Servo ColorIndicator;
    private CRServo flagServo;

    private String IndexMode = "INTAKE";
    private Servo Hood = null;

    //private double CPR = 142; //PPR * 4

    private double revolutions = 0;
    private int revolveTimes = 0;
    private double HoodPos = 0;
    private int trialVel = -1000;
    private TouchSensor stopper = null;
    private boolean hit;
    private double endTime;

    private double bigA;
    private double smallA;
    private int teamTag = 20;

    private boolean IDRegistering = true;

    private boolean OnTarget = false;

    //Color
    private ColorSensor color1;
    private ColorSensor color2;
    private ColorSensor color3;
    private ColorSensor loadcolor;
    private ColorSensor testcolor;
    private String colorName;

    private static final int[] motif1 = {1,2,2};
    private static final int[] motif2 = {2,1,2};
    private static final int[] motif3 = {2,2,1};
    private static final int[] purple = {2,2,2};
    private int[] chosenmotif;
    private int[] currentorder;


    @Override
    public void runOpMode() {
        // Initialize the Limelight 3A
        limelight = hardwareMap.get(Limelight3A.class, "limelight3A");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        /* Starts polling for data */
        limelight.start();

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        LeftFront = hardwareMap.get(DcMotor.class, "LeftFront");
        LeftBack = hardwareMap.get(DcMotor.class, "LeftBack");
        RightFront = hardwareMap.get(DcMotor.class, "RightFront");
        RightBack = hardwareMap.get(DcMotor.class, "RightBack");

        Index = hardwareMap.get(DcMotor.class, "Index");
        stopper = hardwareMap.get(TouchSensor.class, "Stopper");
        Hood = hardwareMap.get(Servo.class, "Hood");
        flagServo = hardwareMap.get(CRServo.class, "Flag");

        Launcher = hardwareMap.get(DcMotorEx.class, "Launcher");
        Pusher = hardwareMap.get(Servo.class, "Pusher");
        ColorIndicator = hardwareMap.get(Servo.class, "Color");
        IMU imu = hardwareMap.get(IMU.class, "imu");
        //Color
        color1 = hardwareMap.get(ColorSensor.class, "Color1");
        color2 = hardwareMap.get(ColorSensor.class, "Color2");
        color3 = hardwareMap.get(ColorSensor.class, "Color3");
        loadcolor = hardwareMap.get(ColorSensor.class, "Load");

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        LeftFront.setDirection(DcMotor.Direction.FORWARD);
        LeftBack.setDirection(DcMotor.Direction.REVERSE);
        RightFront.setDirection(DcMotor.Direction.FORWARD);
        RightBack.setDirection(DcMotor.Direction.REVERSE);

        LeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Index.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //Transfer.setDirection(DcMotor.Direction.FORWARD);
        //Intake.setDirection(DcMotor.Direction.FORWARD);
        Launcher.setDirection(DcMotor.Direction.FORWARD);

        Launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Launcher.setVelocityPIDFCoefficients(2, 25, 16, 1);

        Index.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        chosenmotif = motif1;
        testcolor = color1;
        colorName = "Color1";
        waitForStart();

        runtime.reset();


        boolean LauncherMaxSpd = false;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //constantly update what the color sensors see in regard to order (load position only).
            if(LoadPos()){
                currentorder = new int[] {sensorDetectColor(color1), sensorDetectColor(color2), sensorDetectColor(color3)};
                telemetry.addData("CurrOrder", currentorder);
                telemetry.addLine();
            }

            telemetry.addLine();
            if(teamTag == 20){
                telemetry.addData("Team", "Blue");
            }else if(teamTag == 24){
                telemetry.addData("Team", "Red");
            }
            telemetry.addLine();
            //limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
            LLResult result = limelight.getLatestResult();

            //Limelight
            if (result != null) {
                if (result.isValid()) {
                    Pose3D botpose = result.getBotpose();

                    OnTarget = (result.getFiducialResults().get(0).getFiducialId() == teamTag);
                    telemetry.addData("looking at", result.getFiducialResults().get(0).getFiducialId());

                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("ta", result.getTa());
                    telemetry.addData("Distance", GetDistance(result.getTa()));
                    //telemetry.addData("Botpose", botpose.toString());

                    List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                    for (LLResultTypes.FiducialResult fiducial : fiducials) {
                        int id = fiducial.getFiducialId(); // The ID number of the fiducial

                        telemetry.addData("Fiducial " + id, " " );
                    }

                    if (botpose != null) {
                        double x = botpose.getPosition().x;

                        double y = botpose.getPosition().y;

                        //telemetry.addData("MT1 Location", "(" + truncate(x, 3) + ", " + truncate(y, 3) + ")");
                    }
                }
            }

            int distance = (int)(GetDistance(result.getTa()));

            Pusher.scaleRange(0.125, 0.425);
            //Pusher.setPosition(1);
            //SetIndexMode();
            telemetry.addData("Color: ", sensorDetectColor(color1));
            telemetry.addData("Trial Vel", trialVel);
            telemetry.addData("Attempt vel", -1450 - (200/30 * (distance - 270)));

            if(distance > 265 && result.isValid() && (OnTarget || !IDRegistering)){
                HoodPos = 0.35;
                Hood.setPosition(HoodPos);
                if(result.getTx() < bigA && result.getTx() > smallA){
                    ColorIndicator.setPosition(0.5);

                }else if(result.getTx() > bigA){
                    ColorIndicator.setPosition(0.35);
                }else{
                    ColorIndicator.setPosition(0.6);
                }
                if(gamepad1.y){
                    Launch(3, -1450 - ((200 / 30) * (distance - 270)));
                }

            }else if(distance < 190 && distance > 135 && (OnTarget || !IDRegistering)){

                //ColorIndicator.setPosition(0.28);
                if(result.getTx() < 5 && result.getTx() > -3){
                    ColorIndicator.setPosition(0.5);

                }else if(result.getTx() > 5){
                    ColorIndicator.setPosition(0.35);
                }else{
                    ColorIndicator.setPosition(0.6);
                }
                if(gamepad1.y){
                    Launch(3, -1200 - ((155 / 55) * (distance - 135)));
                }

            }else if(distance < 135 && distance > 120 && (OnTarget || !IDRegistering)){
                //ColorIndicator.setPosition(0.28);
                if(result.getTx() < 5 && result.getTx() > -3){
                    ColorIndicator.setPosition(0.5);
                }else if(result.getTx() > 5){
                    ColorIndicator.setPosition(0.35);
                }else{
                    ColorIndicator.setPosition(0.6);
                }
                if(gamepad1.y){
                    Launch(3, -960 - ((135 / 15) * (distance - 120)));
                }
            }else{
                ColorIndicator.setPosition(0.28);
            }

            //Hood Change close Range
            if(distance > 140 && distance < 190){
                HoodPos = 0.65;
                Hood.setPosition(HoodPos);
            }else if(distance < 130){
                HoodPos = 0.75;
                Hood.setPosition(HoodPos);
            }

            if(gamepad1.rightStickButtonWasReleased()){
                IDRegistering = !IDRegistering;
            }

            if(gamepad2.bWasReleased()){
                testcolor = color1;
                colorName = "Color1";
            }
            if(gamepad2.aWasReleased()){
                testcolor = color2;
                colorName = "Color2";

            }
            if(gamepad2.xWasReleased()){
                testcolor = color3;
                colorName = "Color3";
            }
            if(gamepad2.yWasReleased()){
                testcolor = loadcolor;
                colorName = "LoadColor";
            }

            telemetry.addLine();
            telemetry.addData("ColorSensor", colorName);
            telemetry.addData("Red", testcolor.red());
            telemetry.addData("Green", testcolor.green());
            telemetry.addData("Blue", testcolor.blue());
            telemetry.addLine();
            /*if(gamepad1.leftBumperWasReleased()){
                trialVel += 50;
            }
            if(gamepad1.rightBumperWasReleased()){
                trialVel -= 50;
            }*/

            /*if(gamepad1.xWasReleased()){
                hit = true;
                if(Empty()){
                    revolveTimes = 6;
                }
            }

            if(!Empty() && revolveTimes == 0){
                if(!Arrays.equals(currentorder, chosenmotif)){
                    revolveTimes = 2;
                }
            }

            if(gamepad1.startWasReleased()){
                revolutions = 0;
            }

            if(gamepad1.backWasReleased()){
                revolveTimes = 1;
            }
            //0.2 low, 1 is high
            if(gamepad1.dpadLeftWasReleased() && HoodPos > 0){
                HoodPos -= 0.1;
            }
            if(gamepad1.dpadRightWasReleased() && HoodPos < 1){
                HoodPos += 0.1;
            }

            if(gamepad1.dpad_up){
                flagServo.setPower(-1);
            }else if(gamepad1.dpad_down){
                flagServo.setPower(1);
            }else{
                flagServo.setPower(-0.01);
            }

            Hood.setPosition(HoodPos);

            telemetry.addData("HoodPos", HoodPos);
            telemetry.addData("Touch: ", stopper.isPressed());

            Pusher.setPosition(gamepad1.right_trigger);
            //Launcher.setVelocity(gamepad1.right_trigger * -6800);
            //Launcher.setVelocity(gamepad1.left_trigger * 3800);
            */
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


            // Send calculated power to wheels
            if(gamepad1.left_trigger == 0){
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

            if(teamTag == 20){
                if(gamepad1.leftStickButtonWasPressed()){
                    teamTag = 24;
                }
                bigA = 3.25;
                smallA = -1.5;
            }else{
                if(gamepad1.leftStickButtonWasPressed()){
                    teamTag = 20;
                }
                bigA = 1.5;
                smallA = -3.25;
            }

            // Revolve
             //
             //
            if(gamepad1.left_bumper){
                hit = true;
                Index.setPower(-0.1);
            }else if (revolveTimes == 0){
                Index.setPower(0);
            }
            if(revolveTimes > 0) {
                revolves();
            }

                //ColorIndicator.scaleRange(0.277, 0.5); //(0.277 is red 0.5 is green so it makes the 0-1 red-green)

            //ColorIndicator.setPosition(Math.abs(LauncherVeloc/2200)); //if no velocity it will be red, the more velocity closer to max (2200) will make it closer to green
            //Tolerance Value.
            LauncherMaxSpd = LauncherVeloc < -1800;

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
            //telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
            telemetry.addData("Motor Speed", "%.2f", LauncherVeloc);
            telemetry.addData("Is Launcher at full speed?", LauncherMaxSpd);
            telemetry.update();
        }

    }
    public static double truncate(double value, int places) {
        double factor = Math.pow(10, places);
        return Math.floor(value * factor) / factor;
    }
    void Launch(int amount, int vel){
        int times = amount;

        if(revolutions % 2 == 0){
            Revolve(1);
        }

        Launcher.setVelocity(vel);
        int vel2 = 0;
        do{

            while(Launcher.getVelocity() > vel){
                Launcher.setVelocity(vel);
            }
            /*if(times == amount){
                while(Launcher.getVelocity() > vel){
                    Launcher.setVelocity(vel);
                }
                vel2 = (int)Launcher.getVelocity();
            }else{
                while(Launcher.getVelocity() > vel2){
                    Launcher.setVelocity(vel2);
                }
            }*/
            waitTime(0.5);
            if(gamepad1.b){
                return;
            }
            Launcher.setVelocity(vel2);
            //waitTime(1);
            Pusher.setPosition(1);
            waitTime(0.5);
            Pusher.setPosition(0);
            waitTime(0.5);
            Revolve(2);
            amount -= 1;

        }while(amount != 0);
        revolveTimes = 1;
    }

    double GetDistance(double TArea){
        return (120.9809 + (331.8667 * Math.pow(Math.E, (-2.119361 * TArea))));
    }

    void Revolve(int times){
        double checkTimer = getRuntime() + 1;
        Index.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        revolutions += times;
        boolean hit = true;
        while(times > 0){

            if(gamepad1.left_bumper){
                hit = true;
                Index.setPower(-0.1);
            }
            else{
                Index.setPower(0.8);
            }

            if(!hit && stopper.isPressed()){
                hit = true;
                times -= 1;
                if(times == 0){
                    Index.setPower(0);
                }
                checkTimer = getRuntime() + 1;
            }
            /*if(getRuntime() > checkTimer){

                while(!stopper.isPressed()){
                    Index.setPower(-0.15);
                }
                Index.setPower(0.15);
                checkTimer = getRuntime() + 1;
            }*/
            if(!stopper.isPressed()){
                hit = false;
            }

        }
        //waitTime(0.15);
        /*while(!stopper.isPressed()){
            Index.setPower(-0.15);
        }*/
        Index.setPower(0);


    }

    boolean Empty(){
        for (int i : currentorder){
            if (i == 0){
                return true;
            }
        }
        return false;
    }
    void SetIndexMode(){
        if((Objects.equals(IndexMode, "INTAKE") && (revolutions % 2 == 0))){
            Revolve(1);
        }

        if((Objects.equals(IndexMode, "LAUNCH")) && (revolutions % 2 == 1)){
            Revolve(1);
        }
    }

    public boolean LoadPos(){
        //int[] load = {724, 158, 340};
        //int[] launch;

        telemetry.addLine();

        return true;
    }

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

        return findLargest(confidenceValues);
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


    void waitTime(double time){
        double end = getRuntime() + time;
        while(getRuntime() < end){

        }
    }

    double Limelight(YawPitchRollAngles orientation){

        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
        LLResult result = limelight.getLatestResult();

        if (result != null) {
            if (result.isValid()) {
                Pose3D botpose = result.getBotpose();
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("ta", result.getTa());
                telemetry.addData("Distance", GetDistance(result.getTa()));
                //telemetry.addData("Botpose", botpose.toString());

                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    int id = fiducial.getFiducialId(); // The ID number of the fiducial

                    telemetry.addData("Fiducial " + id, " " );
                }

                if (botpose != null) {
                    double x = botpose.getPosition().x;

                    double y = botpose.getPosition().y;

                    //telemetry.addData("MT1 Location", "(" + truncate(x, 3) + ", " + truncate(y, 3) + ")");
                }
            }
        }
        return result.getTa();

    }
    public boolean OnTargetAprilTag (){

        //YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        //limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
        LLResult result = limelight.getLatestResult();

        //If it can see the april tag, but it isn't straight ahead, it will try to correct
        if(abs(result.getTx()) >= abs(3)) {
            result = limelight.getLatestResult();
            telemetry.addData("tx", abs(result.getTx()));
            telemetry.update();

            if (result.getTx() < 0) {
                LeftFront.setPower(-0.3);
                LeftBack.setPower(-0.3);
                RightFront.setPower(0.3);
                RightBack.setPower(0.3);
            } else if (result.getTx() > 0){
                LeftFront.setPower(0.3);
                LeftBack.setPower(0.3);
                RightFront.setPower(-0.3);
                RightBack.setPower(-0.3);
            } else {
                LeftFront.setPower(0);
                LeftBack.setPower(0);
                RightFront.setPower(0);
                RightBack.setPower(0);
            }
            return false;
        }
        else{
            LeftFront.setPower(0);
            LeftBack.setPower(0);
            RightFront.setPower(0);
            RightBack.setPower(0);
            return true;
        }
    }

    void Load(){
        if(revolutions % 2 == 1){
            Revolve(1);
        }
        waitTime(0.4);
        Revolve(2);
        waitTime(0.4);
        Revolve(2);
        waitTime(0.4);
        Revolve(2);
    }

    void revolves(){
        if(endTime > getRuntime()){
            Index.setPower(0);
        }else if(!gamepad1.left_bumper) {
            Index.setPower(0.55);
        }

        if (!hit && stopper.isPressed()) {
            hit = true;
            revolveTimes -= 1;
            revolutions += 1;

            if(revolutions % 2 == 0){
                endTime = getRuntime() + 0.75;
            }

            if (revolveTimes == 0) {
                Index.setPower(0);
            }

        }

        if(!stopper.isPressed()){
            hit = false;
        }

    }
}
