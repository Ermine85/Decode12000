package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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


        //leftEncoderMotor = hardwareMap.get(DcMotor.class, "LeftFront");
        //rightEncoderMotor = hardwareMap.get(DcMotor.class, "RightFront");

        waitForStart();

        //Auto Script

        goVroom(16000,0.25f);


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

}
