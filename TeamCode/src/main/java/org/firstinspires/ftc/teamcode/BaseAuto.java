package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Auto", group="Robot")
public class BaseAuto extends LinearOpMode{


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
    private DcMotor Transfer = null;
    private DcMotorEx Launcher = null;
    private Servo LaunchServo = null;
    private DcMotor Encoder = null;

    private DcMotor leftEncoderMotor = null;
    private DcMotor rightEncoderMotor = null;

    @Override
    public void runOpMode(){
        //Variable initiation

        LF = hardwareMap.get(DcMotor.class, "LeftFront");
        LB = hardwareMap.get(DcMotor.class, "LeftBack");
        RF = hardwareMap.get(DcMotor.class, "RightFront");
        RB = hardwareMap.get(DcMotor.class, "RightBack");

        Encoder = hardwareMap.get(DcMotor.class, "Encoder");

        LF.setDirection(DcMotor.Direction.FORWARD);
        LB.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);

        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Transfer = hardwareMap.get(DcMotor.class, "Transfer");
        Launcher = hardwareMap.get(DcMotorEx.class, "Launcher");
        LaunchServo = hardwareMap.get(Servo.class, "launch_servo");

        Transfer.setDirection(DcMotorSimple.Direction.FORWARD);
        Intake.setDirection(DcMotorSimple.Direction.FORWARD);
        Launcher.setDirection(DcMotorSimple.Direction.FORWARD);

        Launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //leftEncoderMotor = hardwareMap.get(DcMotor.class, "LeftFront");
        //rightEncoderMotor = hardwareMap.get(DcMotor.class, "RightFront");

        waitForStart();

        LaunchServo.setPosition(1);

        LaunchServo.scaleRange(0.73, 0.83);

        boolean LauncherMaxSpd = false;


        //Auto Script

        goVroom(16000,0.5f);

        //shoot three balls
        shootBalls(3000);

        //Continue back
        goVroom(10000,0.5f);

        //Turn toward artifacts
        turnRight(600, 0.5f);

        //Drive to artifacts
        goVroom(22000, 0.5f);
    }
    public void shootBalls(int Time){
        Launcher.setVelocity(-2100);

        sleep(1000);
        LaunchServo.setPosition(0);
        Transfer.setPower(0.5);
        Intake.setPower(-1);

        sleep(Time);
        Launcher.setVelocity(0);
        Transfer.setPower(0);
        Intake.setPower(0);
        LaunchServo.setPosition(1);

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
            telemetry.update();
        }

        LF.setPower(0);
        LB.setPower(0);
        RF.setPower(0);
        RB.setPower(0);
    }
    public void turnRight(int Duration, float Speed){

        LF.setPower(Speed);
        LB.setPower(Speed);
        RF.setPower(-Speed);
        RB.setPower(-Speed);

        //Change this later to IMU stuff instead of sleep
        sleep(Duration);

        LF.setPower(0);
        LB.setPower(0);
        RF.setPower(0);
        RB.setPower(0);
    }

    public void turnLeft(int Duration, float Speed){

        LF.setPower(-Speed);
        LB.setPower(-Speed);
        RF.setPower(Speed);
        RB.setPower(Speed);

        sleep(Duration);

        LF.setPower(0);
        LB.setPower(0);
        RF.setPower(0);
        RB.setPower(0);
    }

}
