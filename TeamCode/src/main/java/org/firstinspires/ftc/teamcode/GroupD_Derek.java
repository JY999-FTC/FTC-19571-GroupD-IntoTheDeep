/*
Author: Jerome Yang
Start: 3/29/2025
End: #/##/####
Purpose: FTC 19571 The Robo Brigade Team D testbed robot code.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class GroupD_Derek extends LinearOpMode {

    // Variables
    int target = 0;
    int position = 0;

    // Declarations of hardware, Best practice to declare in class because if declare in runOpMode can only be used in there
    DcMotorEx MotorT;

    Servo ServoT;

    @Override

    public void runOpMode() throws InterruptedException {

        // Declarations of hardware, make sure name & configuration matches
        MotorT = hardwareMap.get(DcMotorEx.class, "MotorT");

        ServoT = hardwareMap.get(Servo.class, "ServoT");


        // Set up motor
        MotorT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorT.setPower(100);
        MotorT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorT.setTargetPosition(0);
        MotorT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorT.setVelocity(500);

        // Set up Servo
        ServoT.setPosition(0);

        // Wait for start button
        waitForStart();

        if (isStopRequested()) return;


        // While loop runs multiple times per second
        while (opModeIsActive()) {

            // move motor
            if (gamepad1.right_stick_x != 0) {
                target += gamepad1.right_stick_x;
                MotorT.setTargetPosition(target);
            }

            // move servo
            if (gamepad1.right_stick_x != 0) {
                position += gamepad1.left_stick_x;
                ServoT.setPosition(position);
            }

            updateTelemetry();

        }// while end

    }// OpMode end


    // this is a method outside of the while loop, can be used inside the loop. Tlelmentry is the display on the right, can display text & images.
    public void updateTelemetry() {
        telemetry.addData("Left JoyStick: ", gamepad1.left_stick_x);
        telemetry.addLine();
        telemetry.addData("Right JoyStick: ", gamepad1.right_stick_x);
        telemetry.addLine();
        telemetry.update();
    }//update telementry end


<<<<<<< HEAD
}// class end
//Derek is a black boi monk
//Jayden has severe autismey
=======
}// class end
>>>>>>> 8d20e5b5b9132c16e4da122016f5da7bcfe90d80
//Derek is a black boi monk//Derek is a black boi monk//Derek is a black boi monk//Derek is a black boi monk//Derek is a black boi monk//Derek is a black boi monk//Derek is a black boi monk//Derek is a black boi monk//Derek is a black boi monk//Derek is a black boi monk//Derek is a black boi monk
//Derek is a black boi monk
//Derek is a black boi monk
//Derek is a black boi monk
//Derek is a black boi monk
//Derek is a black boi monk
//Derek is a black boi monk
//Derek is a black boi monk
//Derek is a black boi monk
//Derek is a black boi monk
//Derek is a black boi monk
//Derek is a black boi monk