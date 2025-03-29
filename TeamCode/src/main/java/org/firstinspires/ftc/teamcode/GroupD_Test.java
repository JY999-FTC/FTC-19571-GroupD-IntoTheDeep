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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class GroupD_Test extends LinearOpMode {

    // Variables
    double motorPosition = 0;
    double servoPosition = 0;

    // Declarations of hardware, Best practice to declare in class because if declare in runOpMode can only be used in there

    // This is declared with a encoder to track # of rotations. DcMotor for no encoder
    DcMotorEx MotorT;
    //DcMotor MotorT;

    // This is declared as a servo, which moves to a position, it could also be CRServo which is a Continuous Servo. The Servo can be programmed between the 2 modes
    Servo ServoT;
    //CRServo ServoT;

    @Override

    public void runOpMode() throws InterruptedException {

        // Declarations of hardware, make sure name & configuration matches
        MotorT = hardwareMap.get(DcMotorEx.class, "MotorT");
        //MotorT = hardwareMap.get(DcMotor.class, "MotorT"); //For no encoder motor

        ServoT = hardwareMap.get(Servo.class, "ServoT");
        //ServoT = hardwareMap.get(CRServo.class, "ServoT"); //For Continuous Servo


        // Set up motor
        MotorT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the encoder to 0
        MotorT.setPower(1); // Power is the electricity given. from -1(reverse full power) to 1 (full power)
        MotorT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // if no electricity then use power to stop motor. Good for linear slidwe
        MotorT.setTargetPosition(0); // Target position of robo is 0
        MotorT.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Spin to the target position
        MotorT.setVelocity(2000); // Uses Encoder to detect and adjust velocity to constant a RPM

        // Set up Servo
        ServoT.setPosition(0); // Reset the servo, if CRServo then will spin

        // Wait for start button
        waitForStart();

        // If not ready then do not run code below, return to top and run code again
        if (isStopRequested()) return;

        
        // While loop runs multiple times per second
        while (opModeIsActive()) {

            // Move Motor
            if (gamepad1.right_stick_x != 0) { // if statement, there is 2 gamepads and gamepad1.right_stick_x means a # between -1 to 1
                motorPosition += gamepad1.right_stick_x;
                MotorT.setTargetPosition((int) motorPosition);// Motor has a encoder so can move to a target position (int) to get rid of decimals
            }

            // Move no Encoder Motor
            /*
            if (gamepad1.right_stick_x != 0) { // if statement, there is 2 gamepads and gamepad1.right_stick_x means a # between -1 to 1
                MotorT.setPower((gamepad1.right_stick_x);// Motor has a encoder so can move to a target position (int) to get rid of decimals
            }
            */

            // Move Servo
            if (gamepad1.left_stick_x != 0) {
                servoPosition += gamepad1.left_stick_x * 0.002;
                ServoT.setPosition(servoPosition);// position only for servo, CRServo is setDirection()
            }

            // Move CRServo
            /*
            if (gamepad1.left_stick_x != 0) {
                ServoT.setPower(gamepad1.left_stick_x);// position only for servo, CRServo is setDirection()
            }
            */


            // This is a Method, they are defined and can be used multiple times to save time, energy and space
            updateTelemetry();

        }// while end

    }// OpMode end


    // This is a method outside of the while loop, can be used inside the loop. Telemetry is the display on the right.
    public void updateTelemetry() {
        telemetry.addData("Left JoyStick: ", gamepad1.left_stick_x);
        telemetry.addLine();
        telemetry.addData("Right JoyStick: ", gamepad1.right_stick_x);
        telemetry.addLine();
        telemetry.addData("Motor Target Position(Only w/encoder):", motorPosition);
        telemetry.addLine();
        telemetry.addData("Servo Target Position(Only Servo Mode): ", servoPosition);
        telemetry.addLine();
        telemetry.addData("Motor Actual Position:", MotorT.getCurrentPosition());
        telemetry.addLine();
        telemetry.addData("Servo Actual Position: ", ServoT.getPosition());
        telemetry.addLine();
        telemetry.update();
    }//update telemetry end


}// class end
