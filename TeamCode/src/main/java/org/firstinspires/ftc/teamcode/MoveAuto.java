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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Quick Drive", group="Robot")

public class MoveAuto extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor LF   = null;
    private DcMotor LB  = null;
    private DcMotor RF = null;
    private DcMotor RB = null;

    private DcMotor Intake = null;
    private DcMotor Transfer = null;
    private DcMotorEx Launcher = null;
    private Servo LaunchServo = null;

    private boolean Pressed = false;

    private boolean RunLauncer = false;


    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        LF  = hardwareMap.get(DcMotor.class, "LeftFront");
        LB = hardwareMap.get(DcMotor.class, "LeftBack");
        RF = hardwareMap.get(DcMotor.class, "RightFront");
        RB = hardwareMap.get(DcMotor.class, "RightBack");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Transfer = hardwareMap.get(DcMotor.class, "Transfer");
        Launcher = hardwareMap.get(DcMotorEx.class, "Launcher");
        LaunchServo = hardwareMap.get(Servo.class, "launch_servo");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        LF.setDirection(DcMotor.Direction.FORWARD);
        LB.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);

        Transfer.setDirection(DcMotor.Direction.FORWARD);
        Intake.setDirection(DcMotor.Direction.FORWARD);
        Launcher.setDirection(DcMotor.Direction.FORWARD);

        Launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        LaunchServo.setPosition(1);

        LaunchServo.scaleRange(0.73, 0.83);

        boolean LauncherMaxSpd = false;

        if(opModeIsActive()){
            sleep(1000);

            LF.setPower(0.5);
            LB.setPower(0.5);
            RF.setPower(0.5);
            RB.setPower(0.5);

            sleep(1500);

            LF.setPower(0);
            LB.setPower(0);
            RF.setPower(0);
            RB.setPower(0);

            /*Launcher.setVelocity(-2100);

            sleep(1000);
            LaunchServo.setPosition(0);

            sleep( 1500);
            Transfer.setPower(0.5);
            Intake.setPower(-1);

            sleep(2500);

            Launcher.setVelocity(0);
            Transfer.setPower(0);
            Intake.setPower(0);

            sleep(1000);

            LF.setPower(0.5);
            LB.setPower(0.5);
            RF.setPower(0.5);
            RB.setPower(0.5);

            sleep(1500);

            LF.setPower(0);
            LB.setPower(0);
            RF.setPower(0);
            RB.setPower(0);

            sleep(500);

            LF.setPower(0.5);
            LB.setPower(0.5);
            RF.setPower(-0.5);
            RB.setPower(-0.5);

            sleep(500);

            LF.setPower(0);
            LB.setPower(0);
            RF.setPower(0);
            RB.setPower(0);

            sleep(500);

            LF.setPower(0.5);
            LB.setPower(0.5);
            RF.setPower(0.5);
            RB.setPower(0.5);

            sleep(1500);

            LF.setPower(0);
            LB.setPower(0);
            RF.setPower(0);
            RB.setPower(0);

            /*sleep(250);

            LF.setPower(0.5);
            LB.setPower(0.5);
            RF.setPower(0.5);
            RB.setPower(0.5);

            sleep(900);

            LF.setPower(0);
            LB.setPower(0);
            RF.setPower(0);
            RB.setPower(0);
            */
        }
       // while (opModeIsActive()){
       //     double LauncherVeloc = Launcher.getVelocity();

        //    LaunchServo.scaleRange(0.73, 0.83);

            /*
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

            if(gamepad1.b && LauncherMaxSpd || gamepad1.start){
                LaunchServo.setPosition(0);
            }else {
                LaunchServo.setPosition(1);
            }
            */

      //      LauncherMaxSpd = LauncherVeloc < -2100;

        //    telemetry.addData("Is Launcher at full speed?", LauncherMaxSpd);
          //  }
        }



    }

