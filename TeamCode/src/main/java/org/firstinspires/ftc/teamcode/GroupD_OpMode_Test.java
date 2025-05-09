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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class GroupD_OpMode_Test extends LinearOpMode {

    // Motor declarations
    DcMotorEx leftTop_Motor;
    DcMotorEx leftBot_Motor;
    DcMotorEx rightTop_Motor;
    DcMotorEx rightBot_Motor;
    DcMotorEx linearSlide_Motor;


    // Servo declarations
    CRServo leftIntake_Servo;
    CRServo rightIntake_Servo;
    Servo blockIntake_Servo;
    Servo rotateIntake_Servo;
    Servo linearSlide_Servo;
    Servo outtake_Servo;

    // Sensor declarations
    ColorSensor color_Sensor;

    // Variable declarations
    int driveTrain_Factor = 1;

    //@Override
    public void runOpMode() throws InterruptedException {

        leftTop_Motor = hardwareMap.get(DcMotorEx.class, "leftTop_Motor");
        rightTop_Motor = hardwareMap.get(DcMotorEx.class, "rightTop_Motor");
        leftBot_Motor = hardwareMap.get(DcMotorEx.class, "leftBot_Motor");
        rightBot_Motor = hardwareMap.get(DcMotorEx.class, "rightBot_Motor");
        linearSlide_Motor = hardwareMap.get(DcMotorEx.class, "linearSlide_Motor");

        leftIntake_Servo = hardwareMap.get(CRServo.class,"leftIntake_Servo");
        rightIntake_Servo = hardwareMap.get(CRServo.class,"rightIntake_Servo");
        //blockIntake_Servo = hardwareMap.get(Servo.class,"blockIntake_Servo");
        rotateIntake_Servo = hardwareMap.get(Servo.class,"rotateIntake_Servo");
        linearSlide_Servo = hardwareMap.get(Servo.class,"linearSlide_Servo");
        //outtake_Servo = hardwareMap.get(Servo.class,"outtake_Servo");
        color_Sensor = hardwareMap.get(ColorSensor.class,"color_Sensor");

        waitForStart();

        if (isStopRequested()) return;

        while(opModeIsActive()){

            if (gamepad1.dpad_up)
                moveMotor(leftTop_Motor, 50);
            else if (gamepad1.dpad_left)
                moveMotor(leftBot_Motor, 50);
            else if (gamepad1.dpad_right)
                moveMotor(rightTop_Motor, 50);
            else if (gamepad1.dpad_down)
                moveMotor(rightBot_Motor, 50);

            if (gamepad1.x)
                moveCRServo(leftIntake_Servo, 0.5);
            else if (gamepad1.b)
                moveCRServo(rightIntake_Servo, 0.5);
            else if (gamepad1.y)
                moveServo(rotateIntake_Servo, 0.5);
            else if (gamepad1.a)
                moveServo(linearSlide_Servo, 0.5);

            updateTelemetry();

        } // while opModeIsActive end



    }// OpMode end



    /// METHODS

    // Motor Method
    public void moveMotor(DcMotorEx motor, int velocity) {
        motor.setVelocity(velocity);
    }// move Motor end

    // Servo Method
    public void moveServo(Servo servo, double position) {
        servo.setPosition(position);
    }// move Servo end
    public void moveCRServo(CRServo servo, double power) {
        servo.setPower(power);
    }// move Servo end

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
        leftBot_Motor.setPower(frontRightPower * driveTrain_Factor);
        rightTop_Motor.setPower(backLeftPower * driveTrain_Factor);
        rightBot_Motor.setPower(backRightPower * driveTrain_Factor);
    }// controller drive end

    // Telemetry method
    public void updateTelemetry() {
        telemetry.addData("Red: ", color_Sensor.red());
        telemetry.addData("Green: ", color_Sensor.green());
        telemetry.addData("Blue: ", color_Sensor.blue());
        telemetry.addLine();
        telemetry.update();
    }//update telemetry end


}// class end
