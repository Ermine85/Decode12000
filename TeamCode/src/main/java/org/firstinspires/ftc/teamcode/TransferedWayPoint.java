/* Copyright (c) 2019 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


import java.util.Vector;


/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous(name = "Specimin", group = "Auto")
//@Disabled
public class TransferedWayPoint extends LinearOpMode {


    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera


    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "12000RedBaseModel.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/12000RedBaseModel.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "REDdiabolo",
    };
    public int newTarget = 0;
    private DcMotor LeftFrontALE = null;
    private DcMotor LeftBackABE = null;
    private DcMotor RightFront = null;
    private DcMotor RightBackARE = null;


    private DcMotor RightArm = null;
    private DcMotor LeftArm = null;






    private Servo specG = null;


    private Vector<Double> InitialPosition = new Vector<Double>(3);
    private Vector<Double> CurrentPosition = new Vector<Double>(3);


    private Vector<Double> Tolerance = new Vector<>(2);


    private Vector<Double> PreviousEncoder = new Vector<>(3);
    private double FinalDistance = 0;


    private double PreviousIMU = 0;
    private boolean MovingOn = false;
    private ColorSensor ClawSensor, ColorSensor;
    private IMU RobotIMU = null;
    private double StartAngle = 0; //setting starting robot orientation in radians




    private ElapsedTime   runtime = new ElapsedTime();
    private double COUNTS_PER_INCH  = ((Math.PI * 1.25984 * 36) / (2000 * 40));
    //private double COUNTS_PER_INCH  = (((((2.0 * Math.PI * 2.0) / 8192.0) * 2.54 * 18.0) / 70.0) / 18.0 * 28.0) ; // 2pi * wheel radios / encoder tpr


    //Other Variables
    private double DeltaX;
    private double DeltaY;
    private double DeltaA;
    private Boolean Hold;
    private Servo ClawS;


    private String Aliance;


    private double TrackLength = 12.5; //diameter between Left Encoder and Right
    private double BackRadius = 5; //distance between back wheel and the center of L & R


    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    //private TfodProcessor tfod;




    @Override
    public void runOpMode() {


        int position;


        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");


        LeftFrontALE = hardwareMap.get(DcMotor.class, "left_front");
        RightFront = hardwareMap.get(DcMotor.class, "right_front");
        LeftBackABE = hardwareMap.get(DcMotor.class, "left_back");
        RightBackARE = hardwareMap.get(DcMotor.class, "right_back");
        //Vert. Arms
        LeftArm = hardwareMap.get(DcMotor.class, "arm_left");
        RightArm = hardwareMap.get(DcMotor.class, "arm_right");
        //Sensors
        ClawSensor = hardwareMap.get(ColorSensor.class, "ClawSensor");
        ColorSensor = hardwareMap.get(ColorSensor.class, "ColorSensor");


        //Servos
        ClawS = hardwareMap.get(Servo.class, "claw");
        RobotIMU = hardwareMap.get(IMU.class, "imu");


        //Start Encoders
        LeftFrontALE.setDirection(DcMotor.Direction.REVERSE);
        LeftBackABE.setDirection(DcMotor.Direction.FORWARD);
        RightFront.setDirection(DcMotor.Direction.REVERSE);
        RightBackARE.setDirection(DcMotor.Direction.FORWARD);


        LeftArm.setDirection(DcMotor.Direction.REVERSE);


        LeftFrontALE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftBackABE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBackARE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        LeftFrontALE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftBackABE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightBackARE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




        //Get Robots Starting Angle (Robot does not start at angle 0)
        StartAngle = Yaw(true);


        StartVector(InitialPosition, 0, 0, 10);
        StartVector(CurrentPosition, 0, 0,10);  // ^
        StartVector(Tolerance, 1, 2, 1000); //1000 nulls 3rd spot (tolerance only has 2 spots)
        StartVector(PreviousEncoder, 0, 0, 0); //Holds previous encoder counts


        telemetry.update();
        waitForStart();


        if (opModeIsActive()) {




        }






    }   // end runOpMode()




    public void MoveTo(double TargetX, double TargetY, double TargetAngle, double Speed, double timer)
    {
        //TargetAngle = (TargetAngle * (2 * Math.PI) / 360); //Convert target angle to radians
        //double AngleTolerance = Tolerance.get(1) * (2* Math.PI/ 360); //converts tolerance angle from tolerance vector
        //Start with Odoms at 0
        double StartTime = getRuntime();
        double EndTime = getRuntime() + timer;
        double AngleOff = TargetAngle;//Position Variables
        double Crx = 0; //current robot x
        double Cry = 0; //current robot y
        double Cfx = InitialPosition.get(0); //Initial tracks field Pos
        double Cfy = InitialPosition.get(1);


        SetVector(CurrentPosition, InitialPosition.get(0), InitialPosition.get(1), InitialPosition.get(2));
        //Robot Angle and Total Distance to Target
        double RobotYaw = StartAngle - Yaw(true);
        double ThetaF = RobotYaw;
        double RadiusToTarget = Math.sqrt(Math.pow(CurrentPosition.get(0) - TargetX, 2 ) + Math.pow(CurrentPosition.get(1) - TargetY, 2));
        double StartingR = RadiusToTarget;
        double StartingSpeed = Speed;
        while(RadiusToTarget > Tolerance.get(0) || Math.abs(AngleOff) > Tolerance.get(1))  //Total Distance greater than tolerance position
        {
            RobotYaw = StartAngle - Yaw(true);
            //Gets Odometer values (Sets DeltaX and DeltaY)
            GCOV();
            AngleOff = AngleOff - DeltaA;
            //Tracks total distance Traveled
            double TotalDistance = COUNTS_PER_INCH * Math.sqrt(Math.pow(DeltaX, 2) + Math.pow(DeltaY, 2));
            //Get the angle moved based off of robot (Theta Robot)
            double ThetaR = Math.atan(DeltaX / DeltaY);


            //Account for issue with arc-tan since it only returns 0-PI
            if (DeltaY == 0 && DeltaX >= 0) {
                ThetaR = Math.PI / 4;
            } else if (DeltaY == 0 && DeltaX < 0) {
                ThetaR = 5 * Math.PI / 4;
            }
            if (DeltaY < 0) {
                ThetaR = ThetaR + Math.PI;
            }


            //Angle of robot movement in field reference
            ThetaF = ThetaR + RobotYaw;  // Remove RobotYaw for now
            //Distance robot has moved in x-direction
            double Dfx = Math.sin(ThetaF) * TotalDistance;
            //Distance robot has moved in y-direction
            double Dfy = Math.cos(ThetaF) * TotalDistance;


            //track current position
            Crx = Crx + DeltaX;
            Cry = Cry + DeltaY;
            Cfx = Cfx + Dfx;
            Cfy = Cfy + Dfy;


            double DifX = (TargetX - Cfx); //Difference in X
            double DifY = (TargetY - Cfy); //Difference in Y


            //calculate distance from target
            RadiusToTarget = Math.sqrt(Math.pow(DifX, 2) + Math.pow(DifY, 2));
            //Calculate direction to target
            if (DifY == 0)  DifY = 0.001;
            double Ttf = Math.atan(DifX / DifY);
            if (DifY < 0) {
                Ttf = Ttf + Math.PI;
            }
            if (DifY > 0){
                //Do nothing
            }
            Ttf = Ttf + Math.PI;


            double RobotAngle =  Ttf - RobotYaw; //Gets the angle the robot needs to move
            //SpeedRange




            SetWheels(RobotAngle, RobotYaw, Speed, AngleOff);


            telemetry.addData("X", Cfx);
            telemetry.addData("Y", Cfy);
            telemetry.addData("R", RadiusToTarget);


            telemetry.addLine();
            telemetry.addData("Angle Off", AngleOff);
            telemetry.addData("RobotYawVar", RobotYaw * 180/Math.PI);
            telemetry.addData("Yaw", Yaw(false));
            telemetry.update();


            if(getRuntime() >= EndTime)
            {
                LeftFrontALE.setPower(0);
                LeftBackABE.setPower(0);
                RightBackARE.setPower(0);
                RightFront.setPower(0);
                SetVector(InitialPosition, TargetX, TargetY, TargetAngle);
                return;
            }


        }

        SetVector(InitialPosition, Cfx, Cfy, RobotYaw + Math.PI);
        LeftFrontALE.setPower(0);
        LeftBackABE.setPower(0);
        RightBackARE.setPower(0);
        RightFront.setPower(0);
        
    }


    public void SetVector(Vector vector, double X, double Y, double angle )
    {
        vector.set(0, X);
        vector.set(1, Y);
        if(angle != 1000) //Making a null value for when we only need 2 (tolerance)
        {
            vector.set(2, angle);
        }
    }
    public void StartVector(Vector vector, double X, double Y, double angle)
    {
        vector.add(X);
        vector.add(Y);
        if(angle != 1000) //Making a null value for when we only need 2 (tolerance)
        {
            vector.add(angle);
        }


    }


    public void GCOV() //GetCurrentOdomValues
    {
        double LE = LeftFrontALE.getCurrentPosition() - PreviousEncoder.get(0);
        double RE = RightBackARE.getCurrentPosition() - PreviousEncoder.get(1);
        double BE = LeftBackABE.getCurrentPosition() - PreviousEncoder.get(2);
        DeltaX = BE - (BackRadius/(TrackLength/2)) * (RE - LE);

        if(LE - RE == 0)
        {
            LE = 0.0000000001;
        }

        DeltaA = (360*125)/(((2 * Math.PI * (TrackLength/2))/((LE-RE) * COUNTS_PER_INCH)) * 180);

        DeltaY = ((LE + RE) / 2);

        //double DltX = BE;
        SetVector(PreviousEncoder, LeftFrontALE.getCurrentPosition(), RightBackARE.getCurrentPosition(), LeftBackABE.getCurrentPosition());

       /*telemetry.addData("LE", LE);
       telemetry.addData("RE", RE);
       telemetry.addData("BE", BE);
       telemetry.addLine();
       telemetry.addData("DeltaY", DeltaY);
       telemetry.addData("DeltaX", DeltaX);
       telemetry.addData("DeltaA", DeltaA);*/
    }


    public double Yaw(boolean radians)
    {
        if(radians)
        {
            return RobotIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        }else { return RobotIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);}
    }
    public void SetWheels(double RobotAngle, double RobotYaw, double Speed, double AngleOff) //Angle Not accounted For
    {
        if(AngleOff > 5)
        {
            AngleOff = 5;
        }else if(AngleOff < -5)
        {
            AngleOff = -5;
        }
        double F = 1;//ADD + 0.5*(RobotYaw - TargetAngle)
        double M1 = F*(Math.sin(RobotAngle) + Math.cos(RobotAngle))+ (0.075 * -AngleOff); //LF
        double M2 = F*(Math.sin(RobotAngle) - Math.cos(RobotAngle))+ (0.075 * -AngleOff);//RF
        double M3 = F*(-Math.sin(RobotAngle) + Math.cos(RobotAngle))+ (0.075 * -AngleOff);//LB
        double M4 = F*(-Math.sin(RobotAngle) - Math.cos(RobotAngle)) + (0.075 * -AngleOff);//RB
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


        //Sets Motors Power to the powers above
        LeftFrontALE.setPower(M1 * Speed);
        RightFront.setPower(M2 * Speed);
        LeftBackABE.setPower(M3 * Speed);
        RightBackARE.setPower(M4 * Speed);

    }

    public static double findLargest(double[] array) {
        double max = array[0];
        for (int i = 1; i < array.length; i++) {
            if (array[i] > max) {
                max = array[i];
            }
        }
        return max;
    }






}   // end class
// around the world x100000, 14000000 BPM is the craziest of all crazies

