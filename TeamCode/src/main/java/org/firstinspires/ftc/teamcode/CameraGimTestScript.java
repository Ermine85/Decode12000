/* Copyright (c) 2023 FIRST. All rights reserved.
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
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import java.util.List;
import java.util.Vector;

/*
 * This OpMode illustrates the basics of AprilTag recognition and pose estimation, using
 * the easy way.
 *
 * For an introduction to AprilTags, see the FTC-DOCS link below:
 * https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
 *
 * In this sample, any visible tag ID will be detected and displayed, but only tags that are included in the default
 * "TagLibrary" will have their position and orientation information displayed.  This default TagLibrary contains
 * the current Season's AprilTags and a small set of "test Tags" in the high number range.
 *
 * When an AprilTag in the TagLibrary is detected, the SDK provides location and orientation of the tag, relative to the camera.
 * This information is provided in the "ftcPose" member of the returned "detection", and is explained in the ftc-docs page linked below.
 * https://ftc-docs.firstinspires.org/apriltag-detection-values
 *
 * To experiment with using AprilTags to navigate, try out these two driving samples:
 * RobotAutoDriveToAprilTagOmni and RobotAutoDriveToAprilTagTank
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp(name = "CamTest", group = "Concept")

public class CameraGimTestScript extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private Servo XServo = null;
    private Servo YServo = null;

    private double LastUpdate = 0;

    private double wantedDistance;
    private Vector<Double> previous = new Vector<>(3);

    private Vector<Double> current = new Vector<>(3);
    private int ScanPos = 0;

    private boolean TagFound = false;
    private double ServoPos = 0;
    private double curDAX = -100; // current delta angle
    private double curDAY;
    private double TargetPos;

    //These are the motors
    private DcMotor LeftFront = null;
    private DcMotor LeftBack = null;
    private DcMotor RightFront = null;
    private DcMotor RightBack = null;

    private DcMotor Intake = null;
    private DcMotor Storage = null;
    private DcMotor Launcher = null;

    private int[] aprilTagId = new int[3];

    private double RobotAngle = 0;
    private double RobotX = 0;
    private double RobotY = 0;
    private double RobotZ = 0;

    private double AprilTagX = 0;
    private double AprilTagY = 0;
    private double AprilTagZ = 0;

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        LeftFront = hardwareMap.get(DcMotor.class, "LeftFront");
        LeftBack = hardwareMap.get(DcMotor.class, "LeftBack");
        RightFront = hardwareMap.get(DcMotor.class, "RightFront");
        RightBack = hardwareMap.get(DcMotor.class, "RightBack");
        //Intake = hardwareMap.get(DcMotor.class, "Intake");
        //Storage = hardwareMap.get(DcMotor.class, "Storage");
        //Launcher = hardwareMap.get(DcMotor.class, "Launcher");

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

        //Storage.setDirection(DcMotor.Direction.FORWARD);
        //Intake.setDirection(DcMotor.Direction.FORWARD);
        //Launcher.setDirection(DcMotor.Direction.FORWARD);

        //Set Tags
        aprilTagId[0] = 22; //Change? Blue
        aprilTagId[1] = 21; //Change? Red
        aprilTagId[2] = 23; //Change? middle


        StartVector(previous, 0, 0, 0);

        StartVector(current, 0, 0, 0);

        XServo = hardwareMap.get(Servo.class, "x_servo");
        YServo = hardwareMap.get(Servo.class, "y_servo");

        initAprilTag();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");

        telemetry.update();


        waitForStart();

        if (opModeIsActive()) {
            //VERY IMPORTANT LINE: DO NOT EDIT
            /*
            0.555776        0.777888     1
               0.055528 -> 1/4
            0.222112 = 1
             */
            XServo.scaleRange(0.555776 - 0.018509,0.777888 - 0.018509);

            XServo.setPosition(0.5);

            while (opModeIsActive()) {

                telemetry.addData("Servo", XServo.getPosition());


                // Push telemetry to the Driver Station.
                telemetry.update();

                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }


                if(gamepad1.x){
                    telemetryAprilTag(0);
                    GimTrack();
                }

                if (gamepad1.a){
                    XServo.setPosition(0.5);
                }

                //telemetry.addLine(String.format("CurrentVector:", current));

                if (gamepad1.start){
                    MoveToDistance(0, 23.4, 0.5);
                }



                // Share the CPU.
                sleep(20);
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end method runOpMode()

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }

    }   // end method initAprilTag()

    /**
     * Add telemetry about AprilTag detections.
     */
    private void moveToTarget(double tarX, double tarY, double ServoStartAngel) {

    }
    private void telemetryAprilTag(int Id) {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        TagFound = !currentDetections.isEmpty();

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if ((detection.metadata != null && detection.metadata.id == Id) || (detection.metadata != null && Id == 0)) {

                SetVector(current, detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z);

                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));

                AprilTagX = detection.ftcPose.x;
                AprilTagY = detection.ftcPose.y;
                AprilTagZ = detection.ftcPose.z;

            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                curDAX = -100; // reset number
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()


    public void SetVector(Vector vector, double X, double Y, double Z)
    {
        vector.set(0, X);
        vector.set(1, Y);
        vector.set(2, Z);
    }

    public void StartVector(Vector vector, double X, double Y, double Z){
        vector.add(X);
        vector.add(Y);
        vector.add(Z);
    }

    public void GimTrack(){
        double DeltaX = 0 + current.get(0);
        double DeltaY = Math.abs(current.get(1));
        double DeltaZ = Math.abs(current.get(2));

        double DeltaAY = Math.atan(DeltaZ/DeltaY);
        double DeltaAX = Math.atan(DeltaX/DeltaY);

        telemetry.addData("DeltaAX", DeltaAX);
        telemetry.addData("DeltaAY", DeltaAY);

        double ServoAX = DeltaAX / (2*Math.PI);
        double ServoAY = DeltaAY/(80*Math.PI/180);


        //Logic Behind Angle Updates.
        //If seeing after not having seen it

        if(curDAX == -100){
            curDAX = ServoAX;
            curDAY = ServoAY;
            //XServo.setPosition(XServo.getPosition() + curDA);
        }

        if(ServoAX < 0){ // If negative angle (tolerance of 0.005 both positive and negative)
            if(curDAX > 0){ //If it was originally positive angle (means it got to where it needed).
                curDAX = ServoAX;
                XServo.setPosition(XServo.getPosition() + curDAX); //Gets rid of margin
            }

            if(ServoAX < curDAX){ //If the picture gets farther away from 0 and the current angle.
                double change = ServoAX - (curDAX * 0.8); //Get how much father + a small margin
                XServo.setPosition(XServo.getPosition() + change);
                curDAX = ServoAX;
            }
        }
        if(ServoAY < 0){ // If negative angle (tolerance of 0.005 both positive and negative)
            if(curDAY > 0){ //If it was originally positive angle (means it got to where it needed).
                curDAY = ServoAY;
                YServo.setPosition(YServo.getPosition() + curDAY); //Gets rid of margin
            }

            if(ServoAY < curDAY){ //If the picture gets farther away from 0 and the current angle.
                double change = ServoAY - (curDAY * 0.8); //Get how much father + a small margin
                YServo.setPosition(YServo.getPosition() + change);
                curDAY = ServoAY;
            }
        }

        if(ServoAX > 0){ // If positive angle (tolerance of 1 both positive and negative)
            if(curDAX < 0){ //If it was originally negative angle (means it got to where it needed to be).
                curDAX = ServoAX;
                XServo.setPosition(XServo.getPosition() + curDAX);
            }

            if(ServoAX > curDAX){ //If the picture gets farther away from 0.
                double change = ServoAX - (curDAX * 0.8); //Get how much father
                XServo.setPosition(XServo.getPosition() + change);
                curDAX = ServoAX;
            }
        }
        if(ServoAY > 0){ // If positive angle (tolerance of 1 both positive and negative)
            if(curDAY < 0){ //If it was originally negative angle (means it got to where it needed to be).
                curDAY = ServoAY;
                YServo.setPosition(YServo.getPosition() + curDAY);
            }

            if(ServoAY > curDAY){ //If the picture gets farther away from 0.
                double change = ServoAY - (curDAY * 0.8); //Get how much father
                YServo.setPosition(YServo.getPosition() + change);
                curDAY = ServoAY;
            }
        }

        telemetry.addData("ServoAX", ServoAX);

        previous = current;
    }

    public void MoveToDistance(double distanceX, double distanceY, double speed){

        double servoPos = XServo.getPosition();
        double deltaX = distanceX - current.get(0);
        double deltaY = distanceY - current.get(0);

        double angle = 0;
        double deltaA = 0;
        double distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));

        //0.1 is tolerance
        while(Math.abs(distance) > 0.1){

            //Update Values Needed In loop
            telemetryAprilTag(0); //Edits current
            GimTrack();

            deltaX = distanceX - current.get(0);
            deltaY = distanceY - current.get(1);
            distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
            angle = Math.PI - (ServoPos * 2 * Math.PI);

            deltaA = Math.atan(deltaX/deltaY);
            angle += deltaA;

            //Motor Power
            double F = 1;//ADD + 0.5*(RobotYaw - TargetAngle)
            double M1 = F*(Math.sin(angle) + Math.cos(angle)); //LF
            double M2 = F*(Math.sin(angle) - Math.cos(angle));//RF
            double M3 = F*(-Math.sin(angle) + Math.cos(angle));//LB
            double M4 = F*(-Math.sin(angle) - Math.cos(angle));//RB
            double Mmax;
            //Gets the Highest Value of the 4
            Mmax = Math.max(M1, M2);
            Mmax = Math.max(Mmax, M3);
            Mmax = Math.max(Mmax, M4);
            //Scales all Values by Highest to give all a value from 0-1
            if(Mmax > 1)
            {
                M1 = M1/Mmax;
                M2 = M2/Mmax;
                M3 = M3/Mmax;
                M4 = M4/Mmax;
            }

            RightBack.setPower(M4);
            LeftBack.setPower(M3);
            LeftFront.setPower(M1);
            RightFront.setPower(M2);
            //Code to set motors to speeds above.
        }

        RightBack.setPower(0);
        LeftBack.setPower(0);
        LeftFront.setPower(0);
        RightFront.setPower(0);


    }
}   // end class
