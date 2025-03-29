package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class GroupD_Test extends LinearOpMode {

    int target = 0;

    int position = 0;

    DcMotorEx MotorT;

    Servo ServoT;

    @Override

    public void runOpMode() throws InterruptedException {

        MotorT = hardwareMap.get(DcMotorEx.class, "MotorT");

        ServoT = hardwareMap.get(Servo.class, "ServoT");


        MotorT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorT.setPower(0);
        MotorT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorT.setTargetPosition(0);
        MotorT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorT.setVelocity(0);

        ServoT.setPosition(0);


        waitForStart();

        if (isStopRequested()) return;

        MotorT.setPower(100);
        MotorT.setVelocity(100);


        while (opModeIsActive()) {

            target += gamepad1.right_stick_x;
            MotorT.setTargetPosition(target);

            position += gamepad1.left_stick_x;
            ServoT.setPosition(position);


        }



    }// OpMode end

}
