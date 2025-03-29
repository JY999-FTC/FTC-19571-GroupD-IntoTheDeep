package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


public class GroupD_Test extends LinearOpMode {

    DcMotorEx MotorT;

    Servo ServoT;



    public void runOpMode() throws InterruptedException {

        MotorT = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");


    }// OpMode end

}
