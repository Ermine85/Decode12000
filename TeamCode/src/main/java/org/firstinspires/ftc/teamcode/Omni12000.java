/* Copyright (c) 2017 FIRST. All rights reserved.
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


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;


//import org.checkerframework.checker.units.qual.Angle;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


import java.util.ArrayList;
import java.util.List;


//import org.firstinspires.ftc.robotcontroller.external.samples.RobotHardware;




@TeleOp(name="Omni", group="Linear Opmode")
//@Disabled
public class Omni12000 extends LinearOpMode {


   private DcMotor LeftFront = null;
   private DcMotor LeftBack = null;
   private DcMotor RightFront = null;
   private DcMotor RightBack = null;


   private ColorSensor ColorSensor = null, CS = null; //Color is for Intake, Claw is for Claw
   private TouchSensor touch = null, verttouch = null, armTouch = null;


   private Robot12000 Functions = null;
   private boolean BackUpDrive = false;


   //private boolean ArmMode = false;
   private double armSpeed = 1;


   private double ServoPos = 0;


   private int ArmTarget = 0;
   private boolean toggleReadyB = true, toggleReadyT = true; //Trigger and Buttons


   // Declare OpMode members.
   private ElapsedTime runtime = new ElapsedTime();
   private double RobotStartAngle = 0;
   private boolean MoveToTarget = true;
   private double CurrentRobotAngle = 0;
   private boolean HasCube = false;




   private boolean ClawOpen = false, ClawLooking = false;


   private String IntakeMode = "INTAKE";


   private String CurrentCube = "";
   private String WantedCube = "RED";
   private String AfterCube = "NONE";
   private Boolean ReversDrive = false;
   private Boolean Hold = false;


   //Timers
   private double REJECTTIMER = 0, TRANSFERTIMER = 0, CUBEDELAY = 0.25;


   private double FieldAngle = 0;






   private Boolean ReturningHSlides = false, ReturningVert = false, ReturningArm = false;
   //    private String ArmPosition = "NORMAL"; //DOWN, PLANE, NORMAL
   private IMU Imu = null;




   //Motors F = Front B = Back
   //Robot12000 RobotFunctions = new Robot12000(this);


   public Thread slowIntakeThread = null;
   public static double findLargest(double[] array) {
       double max = array[0];
       for (int i = 1; i < array.length; i++) {
           if (array[i] > max) {
               max = array[i];
           }
       }
       return max;
   }
   @Override
   public void runOpMode() {


       telemetry.addData("Status", "Initialized");
       Imu = hardwareMap.get(IMU.class, "imu");
       telemetry.update();
       Functions = new Robot12000(this);
       Imu.initialize(
               new IMU.Parameters(
                       new RevHubOrientationOnRobot(
                               RevHubOrientationOnRobot.LogoFacingDirection.UP, //Control is UP Expansion Down
                               RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                       )
               )


       );
       Functions.init();
       Imu.resetYaw();


       // Wait for the game to start (driver presses PLAY)
       waitForStart();
       runtime.reset();


       ColorSensor = hardwareMap.get(ColorSensor.class, "ColorSensor");
       CS = hardwareMap.get(ColorSensor.class, "ClawSensor");
       touch = hardwareMap.get(TouchSensor.class, "SlideTouch");
       verttouch = hardwareMap.get(TouchSensor.class, "vert_touch");
       armTouch = hardwareMap.get(TouchSensor.class, "arm_touch");


       List<Integer> list = new ArrayList<>();
       List<Integer> GreenList = new ArrayList<>();
       List<Integer> BlueList = new ArrayList<>();


       // Define the task to add red color sensor data to the list


       // run until the end of the match (driver presses STOP)
      while (opModeIsActive()) {
          telemetry.addData("Wanting Cube", WantedCube);
          telemetry.addData("After Cube", AfterCube);
          telemetry.addData("Cube is", CurrentCube);


          CheckColor(false);


          double max; //Used to compare wheel power
          CurrentRobotAngle = Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);


          CurrentRobotAngle *= -1;


          /*if(CurrentRobotAngle < 0)
          {
              CurrentRobotAngle += Math.PI * 2;
          }*/
          // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.


          double axial   =  -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value --
          double lateral =  gamepad1.left_stick_x;


          /*(ReversDrive)
          {
              axial *= -1;
              lateral *= -1;
          }*/
          double yaw = -gamepad1.right_stick_x/1.5;






          telemetry.addData("yaw", yaw);
          double Speed = Math.sqrt(Math.pow(axial, 2) + Math.pow(lateral, 2));


          //FieldAngle = Math.atan(lateral/axial);
          if (axial == 0)  axial = 0.001;


          FieldAngle = Math.atan(lateral / axial);


          if (axial < 0) {
              FieldAngle = FieldAngle + Math.PI;
          }
          if(axial < 0 && lateral < 0)
          {
              FieldAngle -= Math.PI * 2;
          }
          if(FieldAngle < 0)
          {
              FieldAngle += Math.PI * 2;
          }
          if (axial > 0){
              //Do nothing
          }


          //FieldAngle = FieldAngle + Math.PI;


          if(gamepad1.x && gamepad1.start)
          {
              BackUpDrive = true;
          }


           telemetry.addLine();


          /*telemetry.addData("lateral", lateral);
          telemetry.addData("axial", axial);
          telemetry.addLine();
          telemetry.addData("IMU Angle", CurrentRobotAngle*180/Math.PI);
          telemetry.addData("Angle", 360*FieldAngle/(2 * Math.PI));
          telemetry.addLine();*/ // Telemetry for Headless
          //Angle for the robot wheels to count for
          double RobotAngle = FieldAngle - CurrentRobotAngle;
          //Wheel math
          double leftFrontPower = (((Math.sin(RobotAngle) + Math.cos(RobotAngle)) * Speed) - yaw); //LF
          double rightFrontPower = (((Math.sin(RobotAngle) - Math.cos(RobotAngle)) * Speed) - yaw); //RF
          double leftBackPower = (((-Math.sin(RobotAngle) + Math.cos(RobotAngle)) * Speed) - yaw); //LB
          double rightBackPower = (((-Math.sin(RobotAngle) - Math.cos(RobotAngle)) * Speed) - yaw); //RB




          //Get Backup Drive Numbers


          double BleftFrontPower  = axial + lateral - yaw ;
          double BrightFrontPower = -axial + lateral - yaw;
          double BleftBackPower   = axial - lateral - yaw ;
          double BrightBackPower  = -axial - lateral - yaw;


          double backUpMax;


          backUpMax = Math.max(Math.abs(BleftFrontPower), Math.abs(BrightFrontPower));
          backUpMax = Math.max(backUpMax, Math.abs(BleftBackPower));
          backUpMax = Math.max(backUpMax, Math.abs(BrightBackPower));


          if(backUpMax > 1.0)
          {
              BleftFrontPower /= backUpMax;
              BrightFrontPower /= backUpMax;
              BleftBackPower /= backUpMax;
              BrightBackPower /= backUpMax;
          }


          // Normalize the values so no wheel power exceeds 100%
          // This ensures that the robot maintains the desired motion.
          max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
          max = Math.max(max, Math.abs(leftBackPower));
          max = Math.max(max, Math.abs(rightBackPower));
          //Keeps all motor powers the same scale but from 0-1
          if (max > 1.0) {
              leftFrontPower  /= max;
              rightFrontPower /= max;
              leftBackPower   /= max;
              rightBackPower  /= max;
          }


          if(!BackUpDrive)
          {
              Functions.Move(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
          } else if(BackUpDrive) {
              Functions.Move(BleftFrontPower, BrightFrontPower, BleftBackPower, BrightBackPower);
          }








          if(touch.isPressed() && ReturningHSlides)
          {
              Functions.HorzArm(0);
              ReturningHSlides = false;
          }


          if(!armTouch.isPressed() && ReturningArm) //ArmTouch is flipped (just like Vertouch)
          {
              Functions.IntakeArmP(0);
              ReturningArm = false;
          }


          if(!gamepad1.a  && !gamepad1.x && !gamepad1.back && !gamepad1.b && !gamepad1.right_stick_button && !gamepad1.left_stick_button)
          {
              toggleReadyB = true;
          }
          if(gamepad1.right_trigger <= 0.05 && gamepad1.left_trigger <= 0.05)
          {
              toggleReadyT = true;
          }


          if(gamepad1.left_stick_button && toggleReadyB)
          {
              ReturnArms();
          }


          /*if(gamepad1.right_trigger > 0.1)
          {
              if(IntakeMode == "INTAKE"){
                  Functions.SetIntake(-gamepad1.right_trigger/3);
              }else Functions.SetIntake(gamepad1.right_trigger/3);


          }else if (gamepad1.left_trigger > 0.1)
          {
              if(IntakeMode == "INTAKE"){
                  Functions.SetIntake(-gamepad1.left_trigger/2);
              }else Functions.SetIntake(gamepad1.left_trigger/2);
          }else {
              Functions.SetIntake(0);
          }*/


          if(gamepad1.right_trigger > 0.1 && toggleReadyT)
          {
              toggleReadyT = false;
              if(AfterCube == "NONE" || AfterCube == "YELLOW")
              {
                  SetAfter(WantedCube);


              }else{
                  AfterCube = "NONE";
                  Functions.SetIntake(0);
              }


          }


          if(gamepad1.right_stick_button && toggleReadyB)
          {
              if(Hold)
              {
                 Hold = false;
              }else{
                  Hold = true;
              }
          }


          if(gamepad1.left_trigger > 0.1 && toggleReadyT)
          {
              toggleReadyT = false;
              if(AfterCube != "YELLOW")
              {
                  SetAfter("YELLOW");
              }else {
                  AfterCube = "NONE";
                  Functions.SetIntake(0);
              }
          }






          if(gamepad1.back && toggleReadyB)
          {
              toggleReadyB = false;
              if(WantedCube == "BLUE")
              {
                  WantedCube = "RED";
              }else{
                  WantedCube = "BLUE";
              }
          }


          if(getRuntime() >= REJECTTIMER + 0.5 && REJECTTIMER != 0)
          {
              REJECTTIMER = 0;
              Functions.BlockServo(0.75);
              Functions.SetIntake(-1);
              if(AfterCube == "NONE")
              {
                  AfterCube = "RED";
              }
          }


          if(getRuntime() >= TRANSFERTIMER + CUBEDELAY && TRANSFERTIMER != 0)
          {
              TRANSFERTIMER = 0;
              Functions.BlockServo(0.685);


          }


          if(gamepad1.x && toggleReadyB)
          { //55 75
              if(!ClawLooking)
              {
                  ClawLooking = true;
              } else {
                  ClawLooking = false;
                  Functions.ClawServo(0.585);


              }
              toggleReadyB = false;
          }


          if(ClawLooking)
          {
              Functions.ClawServo(0.35);
              if(CheckColorClaw() == WantedCube)
              {
                  ClawLooking = false;
                  Functions.ClawServo(0.585);
                  //Functions.VertArm(0.75);
              }
          }


          telemetry.addData("Claw Open", ClawOpen); //Tells Driver whether claw is open or closed
          telemetry.addData("Holding: ", Hold);
           //Driver Display for Arm status (Returning or Not), used for debugging
          /*telemetry.addData("Horz-Returning", ReturningArm);
          telemetry.addData("IntakeArm-Returning", Functions.ArmReturning());
          telemetry.addData("Has Cube", HasCube);*/


          //For Both Extending and Reducing the robot, At least one bumper and button must be pressed at the same time
          //Right Bumper & B -- Intake Arm In
          //Right Bumper & Y -- Horizontal Arm In
          //Left Bumper & B -- Intake Arm Out
          //Left Bumper & Y -- Horizontal Arm Out


          if(gamepad1.right_bumper) //Out of Robot
          {
              if (gamepad1.b) { //Intake Arm
                  Functions.IntakeArmP(1);
              }else{ Functions.IntakeArmP(0);}


              if(gamepad1.y){ //Horz Arm
                  Functions.HorzArm(1);
              }else if(!ReturningHSlides){ Functions.HorzArm(-0.1); }


          }else if(gamepad1.left_bumper) //IntoRobot
          {
              if(gamepad1.b){ //Intake Arm
                  Functions.IntakeArmP(-1);
              }else{ Functions.IntakeArmP(0);}


              if(gamepad1.y){ //Horz Arm
                  Functions.HorzArm(-1);
              }else if (!ReturningHSlides){ Functions.HorzArm(0); }
          }else {
              if(!ReturningHSlides && !ReturningArm){
                  Functions.HorzArm(0);
                  Functions.IntakeArmP(0);
              }


          }
           //Checks to see if Intake has the wrong colored cube
          if(CurrentCube != "EMPTY" && CurrentCube != AfterCube && AfterCube != "NONE")
          {
              if(AfterCube == "RED")
              {
                  AfterCube = "NONE";
              }
              Reject(); // Calls the Reject Function to open the blocker and force the cube out
              //After about a second Reject puts the blocker back
          }


           //Resets IMU Yaw to make headless forward the direction robots faces
          if(gamepad1.start && !gamepad1.x) // Entrance (U) Pointing Away
          {
              Imu.resetYaw(); //IMU needs to have proper config. in order for resetYaw() to work
          }


          //dpad left and right both move the bucket in order to deposit samples
          if(gamepad1.dpad_left || gamepad2.dpad_right)
          {
              if(ClawLooking)
              {
                  ClawLooking = false;
                  Functions.ClawServo(0.585);


              }
              Functions.Bucket(0.15); //Small power for slower deposit
          }else{
              Functions.Bucket(0); //Bucket auto-returns to up-right position once power is 0
          }


           //dpad up and down control slides
          if(gamepad1.dpad_up){
              Functions.VertArm(1); //Full power
          }else if(gamepad1.dpad_down && !gamepad1.a) //Pushing only down (without a)
          {
              Functions.VertArm(-1); //Full power down (allows for adjustability
          }else if(!ReturningVert){
              //0.1 power holds slides in place (stops gravity from effecting them)
              Functions.VertArm(0.1);
          }
           //Checks if a is pressed with dpad_down
          if(gamepad1.a)
          {
              ReturningVert = true; //Sets Returning to true until touch is pressed
          }
           //Makes slides go with full power
          if(ReturningVert)  {Functions.VertArm(-1);}


          if(!verttouch.isPressed()) //verttouch is inverted so !isPressed is in fact pressed
          {
              ReturningVert = false; //Stops Returning
              Functions.VertArm(0); //stops slides
          }




          telemetry.addData("Vert pressed", verttouch.isPressed());




          if(HasCube && !ReturningHSlides && !ReturningArm)
          {
              //
              // Functions.BlockServo(0.49);
              Transfer(-1);


              HasCube = false;
          }




          // Show the elapsed game time and wheel power.
           //telemetry.addData("ArmSpeed", armSpeed);
           telemetry.addData("Back Up", BackUpDrive);
           telemetry.addData("Status", "Run Time: " + runtime.toString());
           //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
           telemetry.update();
      }




   }
   void CheckColor(boolean loop)
   {


       telemetry.addLine();
       telemetry.addData("Red", ColorSensor.red());
       telemetry.addData("Green", ColorSensor.green());
       telemetry.addData("Blue", ColorSensor.blue());


       //Values are Red, Blue, Green
       double[] empty = {64, 80, 99};//64,80,99 //74,120,120
       double[] red = {724, 158, 340};//724,158,340 //896, 170, 407
       double[] blue = {126, 582, 240};//126,582,240 //179, 867, 365
       double[] yellow = {1018, 277, 1193}; //1018,277,1193 //1386, 363, 1724

       double deltaE = Math.sqrt((Math.pow(empty[0] - ColorSensor.red(),2) + (Math.pow(empty[1] - ColorSensor.blue(),2)) + (Math.pow(empty[2] - ColorSensor.green(),2)) ));
       double deltaR = Math.sqrt((Math.pow(red[0] - ColorSensor.red(),2) + (Math.pow(red[1] - ColorSensor.blue(),2)) + (Math.pow(red[2] - ColorSensor.green(),2)) ));
       double deltaB = Math.sqrt((Math.pow(blue[0] - ColorSensor.red(),2) + (Math.pow(blue[1] - ColorSensor.blue(),2)) + (Math.pow(blue[2] - ColorSensor.green(),2)) ));
       double deltaY = Math.sqrt((Math.pow(yellow[0] - ColorSensor.red(),2) + (Math.pow(yellow[1] - ColorSensor.blue(),2)) + (Math.pow(yellow[2] - ColorSensor.green(),2)) ));

       double emptyConfidence = 1 - (deltaE/3)*((1 / (deltaE + deltaR)) + (1/((deltaE + deltaB))) + (1/(deltaE + deltaY)));
       double redConfidence = 1 - (deltaR/3)*((1 / (deltaR + deltaE)) + (1/((deltaR + deltaB))) + (1/(deltaR + deltaY)));
       double blueConfidence = 1 - (deltaB/3)*((1 / (deltaB + deltaE)) + (1/((deltaB + deltaE))) + (1/(deltaB + deltaY)));
       double yellowConfidence = 1 - (deltaY/3)*((1 / (deltaY + deltaE)) + (1/((deltaY + deltaE))) + (1/(deltaY + deltaE)));
       double[] confidenceValues = {emptyConfidence,redConfidence,blueConfidence,yellowConfidence};
       double largestConfidence = findLargest(confidenceValues);


       if(emptyConfidence == largestConfidence){
           CurrentCube = "EMPTY";
       } else if (redConfidence == largestConfidence) {
           CurrentCube = "RED";
       } else if (blueConfidence == largestConfidence) {
           CurrentCube = "BLUE";
       } else if (yellowConfidence == largestConfidence) {
           CurrentCube = "YELLOW";
       }


       if(AfterCube == CurrentCube)
       {
           Functions.SetIntake(0);
           AfterCube = "NONE";


           if(!Hold)
           {
               ReturnToBucket();
           }


       }
   }






   public String CheckColorClaw()
   {

       telemetry.addLine();
       telemetry.addData("Red", CS.red());
       telemetry.addData("Green", CS.green());
       telemetry.addData("Blue", CS.blue());


       String ClawCube = null;
       //Values are Red, Blue, Green
       double[] empty = {64, 80, 99};//64,80,99 //74,120,120
       double[] red = {724, 158, 340};//724,158,340 //896, 170, 407
       double[] blue = {126, 582, 240};//126,582,240 //179, 867, 365
       double[] yellow = {1018, 277, 1193}; //1018,277,1193 //1386, 363, 1724

        //CS = Color Sensor Variable
       //Get overall distance by using the difference of all colors (red, blue, green) in pythagorean set up sqrt(DeltaRed^2 + DeltaBlue^2...
       double deltaE = Math.sqrt((Math.pow(empty[0] - CS.red(),2) + (Math.pow(empty[1] - CS.blue(),2)) + (Math.pow(empty[2] - CS.green(),2))));
       double deltaR = Math.sqrt((Math.pow(red[0] - CS.red(),2) + (Math.pow(red[1] - CS.blue(),2)) + (Math.pow(red[2] - CS.green(),2))));
       double deltaB = Math.sqrt((Math.pow(blue[0] - CS.red(),2) + (Math.pow(blue[1] - CS.blue(),2)) + (Math.pow(blue[2] - CS.green(),2))));
       double deltaY = Math.sqrt((Math.pow(yellow[0] - CS.red(),2) + (Math.pow(yellow[1] - CS.blue(),2)) + (Math.pow(yellow[2] - CS.green(),2))));

       //Get the confidence of each and find the largest. (If the distance is far compared to others then confidence is very low)
       double emptyConfidence = 1 - (deltaE/3)*((1 / (deltaE + deltaR)) + (1/((deltaE + deltaB))) + (1/(deltaE + deltaY)));
       double redConfidence = 1 - (deltaR/3)*((1 / (deltaR + deltaE)) + (1/((deltaR + deltaB))) + (1/(deltaR + deltaY)));
       double blueConfidence = 1 - (deltaB/3)*((1 / (deltaB + deltaE)) + (1/((deltaB + deltaE))) + (1/(deltaB + deltaY)));
       double yellowConfidence = 1 - (deltaY/3)*((1 / (deltaY + deltaE)) + (1/((deltaY + deltaE))) + (1/(deltaY + deltaE)));
       double[] confidenceValues = {emptyConfidence,redConfidence,blueConfidence,yellowConfidence};
       double largestConfidence = findLargest(confidenceValues);


       if(emptyConfidence == largestConfidence){
           ClawCube = "EMPTY";
       } else if (redConfidence == largestConfidence) {
           ClawCube = "RED";
       } else if (blueConfidence == largestConfidence) {
           ClawCube = "BLUE";
       } else if (yellowConfidence == largestConfidence) {
           ClawCube = "YELLOW";
       }


       return ClawCube;
   }


   void ReturnToBucket()
   {
       ReturningHSlides = true;
       ReturningArm = true;
       HasCube = true;
       Functions.HorzArm(-1);
       Functions.IntakeArmP(-0.8);


   }
   void ReturnArms()
   {
       ReturningHSlides = true;
       ReturningArm = true;
       Functions.IntakeArmP(-0.8);
       Functions.HorzArm(-1);


   }


   void Reject()
   {
       Functions.BlockServo(0.685);
       REJECTTIMER = getRuntime();
       Functions.SetIntake(-0.65);


   }
   void Transfer(double speed)
   {
       Functions.SetIntake(speed);
       //TRANSFERTIMER = getRuntime();
       Functions.BlockServo(0.685);


   }


   void SetAfter(String Color)
   {
       Functions.BlockServo(0.75);
       Functions.SetIntake(-1);
       AfterCube = Color;
   }


   void IntakeCube()
   {
       if(AfterCube != "NONE")
       {
           Reject();
       }
   }
}

