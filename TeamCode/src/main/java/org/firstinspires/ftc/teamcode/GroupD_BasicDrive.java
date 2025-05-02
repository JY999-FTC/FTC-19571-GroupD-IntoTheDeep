/*
Author: Jerome Yang
Start: 3/27/2025
End: #/##/####
Purpose: FTC 19571 The Robo Brigade Team D robot chassis code.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class GroupD_BasicDrive extends LinearOpMode {

    // Motor declarations
    DcMotorEx leftTop_Motor;
    DcMotorEx leftBottom_Motor;
    DcMotorEx rightTop_Motor;
    DcMotorEx rightBottom_Motor;


    // Servo declarations
    CRServo leftIntake_Servo;
    CRServo rightIntake_Servo;
    Servo blockIntake_Servo;
    Servo rotateIntake_Servo;
    Servo linearSlide_Servo;
    Servo outtake_Servo;

    // Sensor declarations
    ColorSensor colorSensor;

    // Variable declarations
    int driveTrain_Factor = 1;

    //@Override
    public void runOpMode() throws InterruptedException {

        leftTop_Motor = hardwareMap.get(DcMotorEx.class, "leftTopMotor");
        rightTop_Motor = hardwareMap.get(DcMotorEx.class, "rightTopMotor");
        leftBottom_Motor = hardwareMap.get(DcMotorEx.class, "leftBottomMotor");
        rightBottom_Motor = hardwareMap.get(DcMotorEx.class, "rightBottomMotor");

        leftIntake_Servo = hardwareMap.get(CRServo.class,"leftIntakeServo");
        rightIntake_Servo = hardwareMap.get(CRServo.class,"rightIntakeServo");
        blockIntake_Servo = hardwareMap.get(Servo.class,"blockIntakeServo");
        rotateIntake_Servo = hardwareMap.get(Servo.class,"rotateIntakeServo");
        linearSlide_Servo = hardwareMap.get(Servo.class,"linearSlideServo");
        outtake_Servo = hardwareMap.get(Servo.class,"outtakeServo");
        colorSensor = hardwareMap.get(ColorSensor.class,"colorSensor");

    }// OpMode end



    /// METHODS

    // DriveTrain method
    public void driveTrain(double rightStickX, double rightStickY, double leftStickX) {
        double x = rightStickX * 1.1; // Counteract imperfect strafing
        double y = -rightStickY; // Remember, Y stick value is reversed
        double rx = leftStickX * 0.7; // turning speed

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        leftTop_Motor.setPower(frontLeftPower * driveTrain_Factor);
        leftBottom_Motor.setPower(frontRightPower * driveTrain_Factor);
        rightTop_Motor.setPower(backLeftPower * driveTrain_Factor);
        rightBottom_Motor.setPower(backRightPower * driveTrain_Factor);
    }// controller drive end

    // Telemetry method
    public void updateTelemetry() {
        telemetry.addData("test", "test");
        telemetry.addLine();
        telemetry.update();
    }//update telemetry end


}// class end
