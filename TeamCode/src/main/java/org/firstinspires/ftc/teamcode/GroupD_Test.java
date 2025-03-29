package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


public class GroupD_Test extends LinearOpMode {

    DcMotorEx MotorT;

    Servo ServoT;



    public void runOpMode() throws InterruptedException {

        MotorT = hardwareMap.get(DcMotorEx.class, "MotorT");

        ServoT = hardwareMap.get(Servo.class, "ServoT");


        MotorT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorT.setPower(1);
        MotorT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorT.setTargetPosition(0);
        MotorT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorT.setVelocity(0);

        ServoT.setPosition(0);


        waitForStart();

        if (isStopRequested()) return;


        while (opModeIsActive()) {

            if (gamepad1.right_stick_button || gamepad1.left_stick_button) {

                MotorT.setVelocity(gamepad1.right_stick_x);

            }

            if (gamepad1.left_stick_button) {

                ServoT.setPosition(gamepad1.right_stick_x);

            }

        }



    }// OpMode end

}
