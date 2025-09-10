package org.firstinspires.ftc.teamcode;
//Contains Functions For OpMOde and Autonomous

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot12000 {
    //Wheel Motors (6 total -- 4 omni -- 2 normal) ~Change to 4 mecanum later~
    //Distance per inch (encoder)
    static final double     COUNTS_PER_INCH  = (537.6 * 1) / (2 * 3.1415); //Counts per motor rev * Gear Reduction / Wheel diameter * pi
    private DcMotor LeftFront = null; //left_front
    private DcMotor RightFront = null; //right_front
    private DcMotor LeftBack = null; //left_back
    private DcMotor RightBack = null; //right_back

    private DcMotor LeftArm = null;
    private DcMotor RightArm = null;
    private CRServo HorizontalArm = null;
    private CRServo HorzArm2 = null;

    private DcMotor IntakeArm = null;
    private DcMotor Intake = null;

    private CRServo BucketServo = null;
    private Servo ClawServo = null;
    private Servo BlockServo = null;





    //Uncomment when implemented
    //private ColorSensor IntakeColor = null;


    private boolean FlippedDrive = false;
    private IMU RobotIMU = null;

    //Auto Move Distance
    public int newRightTarget = 0;
    public int newLeftTarget = 0;
    private LinearOpMode OpMode = null;
    public Robot12000(LinearOpMode opMode) { OpMode = opMode; }

    //All Variable/Motor SetUps
    public void init() //On Initialization
    {
        //Wheels + Odometry
        LeftFront = OpMode.hardwareMap.get(DcMotor.class, "left_front");
        RightFront = OpMode.hardwareMap.get(DcMotor.class, "right_front");
        LeftBack = OpMode.hardwareMap.get(DcMotor.class, "left_back");
        RightBack = OpMode.hardwareMap.get(DcMotor.class, "right_back");
        //Sliders
        LeftArm = OpMode.hardwareMap.get(DcMotor.class, "arm_left");
        RightArm = OpMode.hardwareMap.get(DcMotor.class, "arm_right");
        //Intake
        HorizontalArm = OpMode.hardwareMap.get(CRServo.class, "horizontal_arm");
        HorzArm2 = OpMode.hardwareMap.get(CRServo.class, "horz_arm");
        IntakeArm = OpMode.hardwareMap.get(DcMotor.class, "intake_arm");

        LeftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Servos
        Intake = OpMode.hardwareMap.get(DcMotor.class, "intake");
        BucketServo = OpMode.hardwareMap.get(CRServo.class, "bucket_servo");
        ClawServo = OpMode.hardwareMap.get(Servo.class, "claw");
        BlockServo = OpMode.hardwareMap.get(Servo.class, "block_servo");
        //IMU
        RobotIMU = OpMode.hardwareMap.get(IMU.class, "imu");
        //Color Sensor



        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Uncomment when implemented
        //IntakeColor = OpMode.hardwareMap.get(ColorSensor.class, "intake_color");

        //Set Wheels Facing Same Direction
        LeftFront.setDirection(DcMotor.Direction.FORWARD);
        LeftBack.setDirection(DcMotor.Direction.REVERSE);
        RightFront.setDirection(DcMotor.Direction.FORWARD);
        RightBack.setDirection(DcMotor.Direction.REVERSE);

        LeftArm.setDirection(DcMotor.Direction.REVERSE);

        IntakeArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IntakeArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    //Move Function
    public void Move(double LF, double RF, double LB, double RB) //Sets Motors to work using pre-given power values
    {

            RightBack.setPower(RB);
            RightFront.setPower(RF);
            LeftBack.setPower(LB);
            LeftFront.setPower(LF);

    }
    //Arm Function(s)
    public void VertArm(double power) //Run Without Encoder
    {

        LeftArm.setPower(power); //Sets both arms to power
        RightArm.setPower(power);
    }

    public void Arm(double power, int location) //Running using encoder counts (should hold position)
    {


        //LeftArm.setTargetPosition(location); //Location
        RightArm.setTargetPosition(location);

        //LeftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION); //Mode
        RightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LeftArm.setPower(power); //Power
        RightArm.setPower(power);

        while(RightArm.isBusy()) //Pauses until they reach location
        {
            //nothing
        }

        LeftArm.setPower(0);
        RightArm.setPower(0);

    }

    //Intake
    void SetIntake(double power) //Basic Intake Function (Not with implemented color sensor)
    {
        Intake.setPower(power);
    }


    void HorzArm(double power)
    {
        HorizontalArm.setPower(power);
        HorzArm2.setPower(-power);
    }

    void Bucket(double pos) { BucketServo.setPower(pos); }



    void IntakeArmP(double power) { IntakeArm.setPower(power); }

    void ClawServo(double pos)
    {
        ClawServo.setPosition(pos);
    }

    void BlockServo(double pos)
    {

        BlockServo.setPosition(pos);
    }

    void DelayBlock(double pos, double start, double delay)
    {
        while(OpMode.getRuntime() != start + delay)
        {

        }
        BlockServo.setPosition(pos);
    }

    void ReturnArm()
    {
        IntakeArm.setTargetPosition(25);
        IntakeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        IntakeArm.setPower(0.8);

    }

    public Boolean ArmReturning()
    {
        //if(IntakeArm.getCurrentPosition() >= IntakeArm.getTargetPosition() -100 && IntakeArm.getCurrentPosition() <= IntakeArm.getTargetPosition() + 100 && IntakeArm.getMode() == DcMotor.RunMode.RUN_TO_POSITION)
        if(!IntakeArm.isBusy())
        {
            IntakeArm.setPower(0);
            IntakeArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if(IntakeArm.getMode() == DcMotor.RunMode.RUN_TO_POSITION)
        {
            return true;
        }else{
            return false;

        }
    }


}
