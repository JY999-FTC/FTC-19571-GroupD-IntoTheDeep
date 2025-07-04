/*
Author: Jerome Yang
Start: 3/27/2025
End: #/##/####
Purpose: FTC 19571 The Robo Brigade Team D robot test code.
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
import com.qualcomm.robotcore.util.ElapsedTime;

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
    Servo rotateIntake_Servo; // 0-1 (intake - outtake)
    Servo left_LinearSlide_Servo; // 0.1-0.7 (Extended - Collapsed)
    Servo right_LinearSlide_Servo;
    Servo outtake_Servo; // 0.4-0.9 (Down - Up)

    // Sensor declarations
    ColorSensor color_Sensor;

    // Variable declarations
    ElapsedTime runtime = new ElapsedTime(); // time that has passed
    double[] stopTime = new double[20]; // change to array to have multiple timers
    int driveTrain_Factor = 1;
    int linearSlide_Motor_Position = 0;
    double twoIntake_Servo_Power = 0;
    double rotateIntake_Servo_Position = 0.5;
    double left_LinearSlide_Servo_Position = 0.5;
    double right_LinearSlide_Servo_Position = 0.5;
    double outtake_Servo_Position = 0.6;
    String gameElement_Color = "None";

    //@Override
    public void runOpMode() throws InterruptedException {

        // declare hardware by getting configuration
        leftTop_Motor = hardwareMap.get(DcMotorEx.class, "leftTop_Motor");
        leftBot_Motor = hardwareMap.get(DcMotorEx.class, "leftBot_Motor");
        rightTop_Motor = hardwareMap.get(DcMotorEx.class, "rightTop_Motor");
        rightBot_Motor = hardwareMap.get(DcMotorEx.class, "rightBot_Motor");
        linearSlide_Motor = hardwareMap.get(DcMotorEx.class, "linearSlide_Motor");

        leftIntake_Servo = hardwareMap.get(CRServo.class,"leftIntake_Servo");
        rightIntake_Servo = hardwareMap.get(CRServo.class,"rightIntake_Servo");
        blockIntake_Servo = hardwareMap.get(Servo.class,"blockIntake_Servo");
        rotateIntake_Servo = hardwareMap.get(Servo.class,"rotateIntake_Servo");
        left_LinearSlide_Servo = hardwareMap.get(Servo.class,"left_LinearSlide_Servo");
        right_LinearSlide_Servo = hardwareMap.get(Servo.class,"right_LinearSlide_Servo");
        outtake_Servo = hardwareMap.get(Servo.class,"outtake_Servo");
        color_Sensor = hardwareMap.get(ColorSensor.class,"color_Sensor");

        // reverse because it the only one spinning in wrong direction idk why
        leftTop_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBot_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        //rightTop_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBot_Motor.setDirection(DcMotorSimple.Direction.REVERSE);


        // Reset the linear slide
        linearSlide_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlide_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide_Motor.setPower(1);
        linearSlide_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlide_Motor.setTargetPosition(0);
        linearSlide_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide_Motor.setVelocity(600);

        // When motor has zero power what does it do? BRAKE!!!
        leftTop_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBot_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightTop_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBot_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Stops and dosent run until you press start
        waitForStart();

        // This check if you stop the code then stops the code
        if (isStopRequested()) return;

        while(opModeIsActive()) {

            // gamepad1 is driver and gamepad2 is gunner

            // Drive normally
            driveTrain(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x);

            twoIntake_Servo_Power = gamepad2.right_trigger - gamepad2.left_trigger;

            // determine the color of the sample
            senseColor(color_Sensor.red(), color_Sensor.green(), color_Sensor.blue());

            if (gamepad2.left_bumper && timer(500, 1))
            {
                linearSlide_Motor_Position -= 100;
                timer(0, 1);
            }
            if (gamepad2.right_bumper && timer(500, 2))
            {
                linearSlide_Motor_Position += 100;
                timer(0, 2);
            }
            // Determine Servo Position
            if (gamepad2.x && timer(500, 3))
            {
                rotateIntake_Servo_Position -= 0.1;
                timer(0, 3);
                //sleep(200);
            }

            if (gamepad2.y && timer(500, 4))
            {
                rotateIntake_Servo_Position += 0.1;
                timer(0, 4);
                //sleep(200);
            }
            if (gamepad2.dpad_left && timer(500, 5))
            {
                left_LinearSlide_Servo_Position += 0.1;
                timer(0, 5);
                //sleep(200);
            }

            if (gamepad2.dpad_right && timer(500, 6))
            {
                left_LinearSlide_Servo_Position -= 0.1;
                timer(0, 6);
                //sleep(200);
            }
            if (gamepad2.a && timer(500, 5))
            {
                right_LinearSlide_Servo_Position += 0.1;
                timer(0, 7);
                //sleep(200);
            }

            if (gamepad2.b && timer(500, 6))
            {
                right_LinearSlide_Servo_Position -= 0.1;
                timer(0, 8);
                //sleep(200);
            }
            if (gamepad2.dpad_up && timer(500, 7))
            {
                outtake_Servo_Position += 0.1;
                timer(0, 9);
                //sleep(200);
            }
            if (gamepad2.dpad_down && timer(500, 8))
            {
                outtake_Servo_Position -= 0.1;
                timer(0, 10);
                //sleep(200);
            }

            // Move the Motor
            linearSlide_Motor.setTargetPosition(linearSlide_Motor_Position);

            // Move the Servos
            moveCRServo(leftIntake_Servo, -twoIntake_Servo_Power);
            moveCRServo(rightIntake_Servo, twoIntake_Servo_Power);

            moveServo(rotateIntake_Servo, rotateIntake_Servo_Position);
            moveServo(left_LinearSlide_Servo, left_LinearSlide_Servo_Position);
            moveServo(right_LinearSlide_Servo, right_LinearSlide_Servo_Position);
            moveServo(outtake_Servo, outtake_Servo_Position);



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
        double leftTop_Power = (y + x + rx) / denominator;
        double leftBot_Power = (y - x + rx) / denominator;
        double rightTop_Power = (y - x - rx) / denominator;
        double rightBot_Power = (y + x - rx) / denominator;

        leftTop_Motor.setPower(leftTop_Power * driveTrain_Factor);
        leftBot_Motor.setPower(leftBot_Power * driveTrain_Factor);
        rightTop_Motor.setPower(rightTop_Power * driveTrain_Factor);
        rightBot_Motor.setPower(rightBot_Power * driveTrain_Factor);
    }// controller drive end

    // Use Timer to determine if a certain amount of time has elapsed.
    public boolean timer(double period, int indexOfTimer) {

        if (period == 0) {
            stopTime[indexOfTimer] = runtime.milliseconds();
            return false;
        }
        return runtime.milliseconds() - stopTime[indexOfTimer] > period;
    }// timer end

    public void senseColor(int r, int g, int b ){

        gameElement_Color = "None"; // To Ground R:50 G:90 B:70 Outtake on top R:116 G:200 B:155 to sky R:202 G:337 B:268
        if (r > 830 && g > 950 && b > 125)
            gameElement_Color = "Yellow"; // R:930 G:1050 B: 225
        else if (r > 280 && g > 120 && b > 20)
            gameElement_Color = "Red"; // R:380 G:220 B:120
        else if (r > 0 && g > 90 && b > 320)
            gameElement_Color = "Blue"; // R:100 G:190 B:420
    }// Checked Color

    // Telemetry method
    public void updateTelemetry() {
        telemetry.addData("leftTop_Motor Power: ", leftTop_Motor.getPower());
        telemetry.addData("leftTop_Motor Velocity: ", leftTop_Motor.getVelocity());
        telemetry.addData("leftTop_Motor Position: ", leftTop_Motor.getCurrentPosition());
        telemetry.addLine();
        telemetry.addData("leftBot_Motor Power: ", leftBot_Motor.getPower());
        telemetry.addData("leftBot_Motor Velocity: ", leftBot_Motor.getVelocity());
        telemetry.addData("leftBot_Motor Position: ", leftBot_Motor.getCurrentPosition());
        telemetry.addLine();
        telemetry.addData("rightTop_Motor Power: ", rightTop_Motor.getPower());
        telemetry.addData("rightTop_Motor Velocity: ", rightTop_Motor.getVelocity());
        telemetry.addData("rightTop_Motor Position: ", rightTop_Motor.getCurrentPosition());
        telemetry.addLine();
        telemetry.addData("rightBot_Motor Power: ", rightBot_Motor.getPower());
        telemetry.addData("rightBot_Motor Velocity: ", rightBot_Motor.getVelocity());
        telemetry.addData("rightBot_Motor Position: ", rightBot_Motor.getCurrentPosition());
        telemetry.addLine();
        telemetry.addData("linearSlide_Motor Power: ", linearSlide_Motor.getPower());
        telemetry.addData("linearSlide_Motor Velocity: ", linearSlide_Motor.getVelocity());
        telemetry.addData("linearSlide_Motor Position: ", linearSlide_Motor.getCurrentPosition());
        telemetry.addData("linearSlide_Motor_Position: ", linearSlide_Motor_Position);
        telemetry.addLine();
        telemetry.addData("leftIntake_Servo: ", leftIntake_Servo.getPower());
        telemetry.addData("rightIntake_Servo: ", rightIntake_Servo.getPower());
        telemetry.addLine();
        telemetry.addData("rotateIntake_Servo: ", rotateIntake_Servo.getPosition());
        telemetry.addData("rotateIntake_Servo_Position: ", rotateIntake_Servo_Position);
        telemetry.addLine();
        telemetry.addData("left_LinearSlide_Servo: ", left_LinearSlide_Servo.getPosition());
        telemetry.addData("left_LinearSlide_Servo_Position: ", left_LinearSlide_Servo_Position);
        telemetry.addData("right_LinearSlide_Servo: ", right_LinearSlide_Servo.getPosition());
        telemetry.addData("right_LinearSlide_Servo_Position: ", right_LinearSlide_Servo_Position);
        telemetry.addLine();
        telemetry.addData("outtake_Servo: ", outtake_Servo.getPosition());
        telemetry.addData("outtake_Servo_Position: ", outtake_Servo_Position);
        telemetry.addLine();
        telemetry.addData("sample_Color: ", gameElement_Color);
        telemetry.addData("Red: ", color_Sensor.red());
        telemetry.addData("Green: ", color_Sensor.green());
        telemetry.addData("Blue: ", color_Sensor.blue());
        telemetry.addLine();
        telemetry.update();
    }//update telemetry end


}// class end
