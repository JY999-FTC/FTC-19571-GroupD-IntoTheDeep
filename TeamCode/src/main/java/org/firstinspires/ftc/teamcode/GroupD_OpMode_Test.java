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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

        // declare hardware position
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

        // reverse because it the only one spinning in wrong direction idk
        leftTop_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBot_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        //rightTop_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBot_Motor.setDirection(DcMotorSimple.Direction.REVERSE);


        // Reset the linear slide
        linearSlide_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide_Motor.setPower(1);
        linearSlide_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlide_Motor.setTargetPosition(0);
        linearSlide_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide_Motor.setVelocity(0);

        leftTop_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBot_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightTop_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBot_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        if (isStopRequested()) return;

        while(opModeIsActive()) {

            driveTrain(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x);

            if (gamepad1.dpad_up)
                moveMotor(leftTop_Motor, 50);
            else if (gamepad1.dpad_left)
                moveMotor(leftBot_Motor, 50);
            else if (gamepad1.dpad_right)
                moveMotor(rightTop_Motor, 50);
            else if (gamepad1.dpad_down)
                moveMotor(rightBot_Motor, 50);

            if (gamepad2.right_stick_x != 0){
                moveCRServo(leftIntake_Servo, gamepad1.right_stick_x);
                moveCRServo(rightIntake_Servo, gamepad1.right_stick_x);
            }
            else if (gamepad2.right_stick_y != 0)
                moveServo(rotateIntake_Servo, gamepad1.right_stick_y);
            if (gamepad2.left_stick_x != 0)
                moveServo(linearSlide_Servo, gamepad1.left_stick_x);



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
        telemetry.addData("leftTop_Motor: ", leftTop_Motor.getVelocity());
        telemetry.addData("leftBot_Motor: ", leftBot_Motor.getVelocity());
        telemetry.addData("rightTop_Motor: ", rightTop_Motor.getVelocity());
        telemetry.addData("rightBot_Motor: ", rightBot_Motor.getVelocity());
        telemetry.addLine();
        telemetry.addData("leftIntake_Servo: ", leftIntake_Servo.getPower());
        telemetry.addData("rightIntake_Servo: ", rightIntake_Servo.getPower());
        telemetry.addLine();
        telemetry.addData("rotateIntake_Servo: ", rotateIntake_Servo.getPosition());
        telemetry.addLine();
        telemetry.addData("linearSlide_Servo: ", linearSlide_Servo.getPosition());
        telemetry.addLine();
        telemetry.addData("Red: ", color_Sensor.red());
        telemetry.addData("Green: ", color_Sensor.green());
        telemetry.addData("Blue: ", color_Sensor.blue());
        telemetry.addLine();
        telemetry.update();
    }//update telemetry end


}// class end
