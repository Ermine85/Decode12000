package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class TestBenchIMU {

    IMU imu;
    YawPitchRollAngles Orientation;

    private void init(HardwareMap hwMap){

        imu = hwMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection UsbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(logoDirection,UsbDirection);

        imu.initialize(new IMU.Parameters(RevOrientation));
        imu.resetYaw();
    }
}
