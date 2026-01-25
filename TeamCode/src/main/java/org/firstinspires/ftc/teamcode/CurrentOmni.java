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

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.BatteryChecker;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

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

    private DcMotor Intake = null;
    private DcMotor Transfer = null;
    private DcMotor Index = null;
    private DcMotorEx Launcher = null;
    private Servo Pusher = null;
    private Servo ColorIndicator;

    private boolean Pressed = false;

    private boolean RunLauncer = false;
    private String IndexMode = "INTAKE";
    private Servo Hood = null;

    private double CPR = 142; //PPR * 4

    private double revolutions = 0;
    private double HoodPos = 0;
    private int trialVel = -1000;
    private TouchSensor stopper = null;


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

        Launcher = hardwareMap.get(DcMotorEx.class, "Launcher");
        Pusher = hardwareMap.get(Servo.class, "Pusher");
        ColorIndicator = hardwareMap.get(Servo.class, "Color");
        IMU imu = hardwareMap.get(IMU.class, "imu");

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

        //Transfer.setDirection(DcMotor.Direction.FORWARD);
        //Intake.setDirection(DcMotor.Direction.FORWARD);
        Launcher.setDirection(DcMotor.Direction.FORWARD);

        Launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        Index.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        runtime.reset();


        boolean LauncherMaxSpd = false;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
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

            int distance = (int)(GetDistance(result.getTa()));
            Pusher.scaleRange(0.125, 0.425);
            //SetIndexMode();
            telemetry.addData("Trial Vel", trialVel);
            telemetry.addData("Attempt vel", -1450 - (200/30 * (distance - 270)));

            if(distance > 270 && result.isValid()){
                HoodPos = 0.3;
                Hood.setPosition(HoodPos);
                if(result.getTx() < 3.25 && result.getTx() > -1.5){
                    ColorIndicator.setPosition(0.5);
                }else if(result.getTx() > 3.25){
                    ColorIndicator.setPosition(0.35);
                }else{
                    ColorIndicator.setPosition(0.6);
                }
                if(gamepad1.yWasReleased()){
                    Launch(3, -1450 - (200 / 30 * (distance - 270)));
                }

            }else{
                ColorIndicator.setPosition(0.28);
            }

/*
            if(gamepad1.a){
                LeftFront.setPower(1);
            }else{
                LeftFront.setPower(0);
            }

            if(gamepad1.b){
                LeftBack.setPower(1);
            }else{
                LeftBack.setPower(0);
            }
            if(gamepad1.x){
                RightFront.setPower(1);
            }else{
                RightFront.setPower(0);
            }
            if(gamepad1.y){
                RightBack.setPower(1);
            }else{
                RightBack.setPower(0);
            }*/
            /*if(gamepad1.aWasReleased()){
                Launch(3, -1150);
            }
*/
            if(gamepad1.bWasReleased()){
                Launch(3, trialVel);
            }

            if(gamepad1.leftBumperWasReleased()){
                trialVel += 50;
            }
            if(gamepad1.rightBumperWasReleased()){
                trialVel -= 50;
            }


            if(gamepad1.xWasReleased()){
                Load();
            }

            if(gamepad1.startWasReleased()){
                revolutions = 0;
            }

            if(gamepad1.backWasReleased()){
                Revolve(1);
            }
            //0.2 low, 1 is high
            if(gamepad1.dpadLeftWasReleased() && HoodPos > 0){
                HoodPos -= 0.1;
            }
            if(gamepad1.dpadRightWasReleased() && HoodPos < 1){
                HoodPos += 0.1;
            }

            Hood.setPosition(HoodPos);

            telemetry.addData("HoodPos", HoodPos);
            telemetry.addData("Touch: ", stopper.isPressed());


            Launcher.setVelocity(gamepad1.right_trigger * -6800);
            //Launcher.setVelocity(gamepad1.left_trigger * 3800);

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


            if(times == amount){
                while(Launcher.getVelocity() > vel){
                    Launcher.setVelocity(vel);
                }
                vel2 = (int)Launcher.getVelocity();
            }else{
                while(Launcher.getVelocity() > vel2){
                    Launcher.setVelocity(vel2);
                }
            }
            //waitTime(1);
            Pusher.setPosition(1);
            waitTime(0.5);
            Pusher.setPosition(0);
            waitTime(0.5);
            Revolve(2);
            amount -= 1;

        }while(amount != 0);

    }

    double GetDistance(double TArea){
        return (120.9809 + (331.8667 * Math.pow(Math.E, (-2.119361 * TArea))));
    }

    void Revolve(int times){
        Index.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        revolutions += times;
        boolean hit = true;
        while(times > 0){

            Index.setPower(0.20);
            if(!hit && stopper.isPressed()){
                hit = true;
                times -= 1;
            }
            if(!stopper.isPressed()){
                hit = false;
            }
        }

        Index.setPower(0);

    }

    void SetIndexMode(){
        if((Objects.equals(IndexMode, "INTAKE") && (revolutions % 2 == 0))){
            Revolve(1);
        }

        if((Objects.equals(IndexMode, "LAUNCH")) && (revolutions % 2 == 1)){
            Revolve(1);
        }
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

    void Load(){
        if(revolutions % 2 == 1){
            Revolve(1);
        }
        Revolve(2);
        waitTime(0.25);
        Revolve(2);
        waitTime(0.25);
        Revolve(2);
    }

}
