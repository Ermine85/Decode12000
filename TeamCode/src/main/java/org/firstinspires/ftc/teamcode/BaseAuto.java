package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple; 
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;



@Autonomous(name="Auto", group="Robot")
public class  BaseAuto extends LinearOpMode{


    //Needed for encoders:
    /*
    4 motor variables
    set all to RUN_WITH_ENCODER (or something like that)
    connect one motor to one of the side odom-pods (hardware team can help)
    keep track of witch one is connected. (write it down)
    reset encoder value on encoder motor

    Create a function that:
    1. takes a distance and speed
    2. sets the encoder motor to go that distance
    3. sets motor powers to speed
    4. tracks its current distance (get the encoder count using the motor variable)
    5. stops all speed when it reaches distance.
     */
    //Variable creation

    private DcMotor RF = null;
    private DcMotor LF = null;
    private DcMotor RB = null;
    private DcMotor LB = null;

    private DcMotor Intake = null;

    private double timeOut = 3.00;
    private DcMotor Transfer = null;
    private DcMotorEx Launcher = null;
    private Servo LaunchServo = null;
    private DcMotor Encoder = null;
    private Servo ColorIndicator;


    private DcMotor leftEncoderMotor = null;
    private DcMotor rightEncoderMotor = null;

    IMU imu;
    YawPitchRollAngles Orientation;

    private Limelight3A limelight;
    boolean LauncherMaxSpd = false;

    double slowSpeed = 0.15;



    @Override
    public void runOpMode() {

        //Variable initiation

        limelight = hardwareMap.get(Limelight3A.class, "limelight3A");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        /* Starts polling for data */
        limelight.start();

        LF = hardwareMap.get(DcMotor.class, "LeftFront");
        LB = hardwareMap.get(DcMotor.class, "LeftBack");
        RF = hardwareMap.get(DcMotor.class, "RightFront");
        RB = hardwareMap.get(DcMotor.class, "RightBack");

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Encoder = hardwareMap.get(DcMotor.class, "Encoder");

        LF.setDirection(DcMotor.Direction.FORWARD);
        LB.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);

        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Transfer = hardwareMap.get(DcMotor.class, "Transfer");
        Launcher = hardwareMap.get(DcMotorEx.class, "Launcher");
        LaunchServo = hardwareMap.get(Servo.class, "launch_servo");
        ColorIndicator = hardwareMap.get(Servo.class, "color_indicator");

        Launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Transfer.setDirection(DcMotorSimple.Direction.FORWARD);
        Intake.setDirection(DcMotorSimple.Direction.FORWARD);
        Launcher.setDirection(DcMotorSimple.Direction.FORWARD);

        Launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //leftEncoderMotor = hardwareMap.get(DcMotor.class, "LeftFront");
        //rightEncoderMotor = hardwareMap.get(DcMotor.class, "RightFront");


        //initialized imu :D
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection UsbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(logoDirection, UsbDirection);

        imu.initialize(new IMU.Parameters(RevOrientation));
        imu.resetYaw();
        Orientation = imu.getRobotYawPitchRollAngles();


        waitForStart();

        LaunchServo.scaleRange(0.73, 0.83);

        LaunchServo.setPosition(1);


        //Auto Script


        Intake.setPower(-1);

        goAwayFromAprilTag(135, .5f);

        shootBalls(3000);






    }

    public void shootBalls(int Time){

        Launcher.setVelocity(-3800);
        Transfer.setPower(0.5);


        double LauncherVeloc = Launcher.getVelocity();

        LauncherMaxSpd = LauncherVeloc < -1800;

        while(!LauncherMaxSpd){

            LauncherVeloc = Launcher.getVelocity();

            LauncherMaxSpd = LauncherVeloc < -1800;
        }

        LaunchServo.setPosition(0);
        Transfer.setPower(0.5);


        sleep(Time);
        Launcher.setPower(2);
        Transfer.setPower(0);
        Intake.setPower(0);
        LaunchServo.setPosition(1);
        sleep(500);
        Launcher.setPower(0);



    }

    public void goVroom(int Distance, float Speed){

        Encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //leftEncoderMotor.setTargetPosition(Distance);
        //rightEncoderMotor.setTargetPosition(Distance);

        //leftEncoderMotor.setPower(Speed);
        //rightEncoderMotor.setPower(Speed);

        LF.setPower(Speed);
        LB.setPower(Speed);
        RF.setPower(Speed);
        RB.setPower(Speed);

        while(Encoder.getCurrentPosition() < Distance){
            //something

            telemetry.addData("LFE Value",  Encoder.getCurrentPosition());
            limeLightTelemetry();
            telemetry.update();

        }

        LF.setPower(0);
        LB.setPower(0);
        RF.setPower(0);
        RB.setPower(0);
    }
    public void turnRight(double Speed, double targetAngle, YawPitchRollAngles orientation){
        double yaw = orientation.getYaw();

        telemetry.addData("Yaw", yaw);
        telemetry.addData("Target", targetAngle);
        telemetry.update();

        LF.setPower(Speed);
        LB.setPower(Speed);
        RF.setPower(-Speed);
        RB.setPower(-Speed);

        while(yaw > -targetAngle){
            orientation = imu.getRobotYawPitchRollAngles();
            yaw = orientation.getYaw();
            if(-targetAngle >= - 20){ // if said -targetAngle is greater then -20 then run slower
                LF.setPower(slowSpeed);
                LB.setPower(slowSpeed);
                RF.setPower(-slowSpeed);
                RB.setPower(-slowSpeed);
            }
            else if(yaw <=(-targetAngle * .75)){ // if yaw is less then or equal to said angle then run slow speed
                LF.setPower(slowSpeed);
                LB.setPower(slowSpeed);
                RF.setPower(-slowSpeed);
                RB.setPower(-slowSpeed);
            }
            telemetry.addData("Yaw", yaw);
            telemetry.addData("Target", targetAngle);
            limeLightTelemetry();
            telemetry.update();
        }
        stopPower();
        imu.resetYaw();
    }

    public void turnLeft(double Speed, double targetAngle, YawPitchRollAngles orientation){

        double yaw = orientation.getYaw();

        telemetry.addData("Yaw", yaw);
        telemetry.addData("Target", targetAngle);
        telemetry.update();

        LF.setPower(-Speed);
        LB.setPower(-Speed);
        RF.setPower(Speed);
        RB.setPower(Speed);

        while(yaw < targetAngle){
            orientation = imu.getRobotYawPitchRollAngles();
            yaw = orientation.getYaw();
            if(targetAngle <= 20){ // if target angle is less than  or equal to 20 always run slow
                LF.setPower(-slowSpeed);
                LB.setPower(-slowSpeed);
                RF.setPower(slowSpeed);
                RB.setPower(slowSpeed);
            }
            else if(yaw >= (targetAngle * .75)){// if yaw is greater then or equal to said angle time .75 = 75% then run slow speed
                LF.setPower(-slowSpeed);
                LB.setPower(-slowSpeed);
                RF.setPower(slowSpeed);



                RB.setPower(slowSpeed);
            }

            telemetry.addData("Yaw", yaw);
            telemetry.addData("Target", targetAngle);
            limeLightTelemetry();
            telemetry.update();

        }
        stopPower();
        imu.resetYaw();

    }

    public void stopPower(){ // stops wheels :)

        LF.setPower(0);
        LB.setPower(0);
        RF.setPower(0);
        RB.setPower(0);

    }

    public void limeLightTelemetry(){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
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

                if (botpose != null) {
                    double x = botpose.getPosition().x;

                    double y = botpose.getPosition().y;

                    //telemetry.addData("MT1 Location", "(" + truncate(x, 3) + ", " + truncate(y, 3) + ")");
                }

            }
        }

    }

    public void goTowardAprilTag (int ATDistance, float Speed){

        // Limelight April Tag code first
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                result = limelight.getLatestResult();
                LF.setPower(-Speed);
                LB.setPower(-Speed);
                RF.setPower(-Speed);
                RB.setPower(-Speed);

                while (GetDistance(result.getTa()) > ATDistance) {
                    result = limelight.getLatestResult();
                    telemetry.addData("Distance", GetDistance(result.getTa()));
                    limeLightTelemetry();
                    telemetry.update();

                }


                stopPower();
            }
        }
    }

    public void goAwayFromAprilTag (int ATDistance, float Speed){

        // Limelight April Tag code first
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
        LLResult result = limelight.getLatestResult();

        telemetry.addData("active",0);
        telemetry.update();

        //sleep(3000);

        LF.setPower(Speed);
        LB.setPower(Speed);
        RF.setPower(Speed);
        RB.setPower(Speed);

        double endTime = getRuntime() + timeOut;

        while ((result != null && !result.isValid()) && (getRuntime() < endTime)) {
            result = limelight.getLatestResult();
            telemetry.addData("Distance", GetDistance(result.getTa()));
            limeLightTelemetry();
            telemetry.update();

        }

        stopPower();

        if(getRuntime() > endTime){
            return;
        }



        if (result.isValid()) {
            result = limelight.getLatestResult();
            LF.setPower(Speed);
            LB.setPower(Speed);
            RF.setPower(Speed);
            RB.setPower(Speed);

            while (GetDistance(result.getTa()) < ATDistance) {
                result = limelight.getLatestResult();
                telemetry.addData("Distance", GetDistance(result.getTa()));
                limeLightTelemetry();
                telemetry.update();

            }

            stopPower();


        }
    }
    public void aprilTagAimCorrection (){
        /*
        while (AprilTagXOrSomething != 0) {
            if (AprilTagXOrSomething < 0){
                turn right
            } else if ((AprilTagXOrSomething > 0)) {
                turn left

            }
        }
        */
    }

    double GetDistance(double TArea){
        return (120.9809 + (331.8667 * Math.pow(Math.E, (-2.119361 * TArea))));
    }
}
