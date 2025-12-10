package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    @Override
    public void runOpMode(){
        //Variable initiation

        waitForStart();

        //Auto Script
    }
}
