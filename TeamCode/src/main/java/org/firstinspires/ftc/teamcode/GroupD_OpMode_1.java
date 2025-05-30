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
public class GroupD_OpMode_1 extends LinearOpMode {

    // Motor declarations
    DcMotorEx leftTop_Motor;
    DcMotorEx leftBot_Motor;
    DcMotorEx rightTop_Motor;
    DcMotorEx rightBot_Motor;
    DcMotorEx linearSlide_Motor; // IDK CHECK

    // Servo declarations
    CRServo leftIntake_Servo;
    CRServo rightIntake_Servo;
    Servo blockIntake_Servo;
    Servo rotateIntake_Servo; // 0-1 (intake - outtake)
    Servo linearSlide_Servo; // 0.1-0.7 (Extended - Collapsed)
    Servo outtake_Servo; // 0.4-0.9 (Down - Up)

    // Sensor declarations
    ColorSensor color_Sensor; // No use rn

    // Variable declarations
    public enum State { // State to know what code to run
        INTAKE,
        OUTTAKE,
    }
    State state = State.INTAKE; // a Instance of State
    ElapsedTime runtime = new ElapsedTime(); // time that has passed
    double[] stopTime = new double[2]; // change to array to have multiple timers
    double driveTrain_Factor = 1; // Motor Power Multiplied by this
    int linearSlide_Motor_Position = 100; // Start at Bottom IDK CHECK
    double twoIntake_Servo_Power = 0;
    double rotateIntake_Servo_Position = 0; // Start at Intake 0-1 (intake - outtake)
    double linearSlide_Servo_Position = 0.7; // Start at Collapsed 0.1-0.7 (Extended - Collapsed)
    double outtake_Servo_Position = 0.4; // Start at Down 0.4-0.9 (Down - Up)

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
        //blockIntake_Servo = hardwareMap.get(Servo.class,"blockIntake_Servo");
        rotateIntake_Servo = hardwareMap.get(Servo.class,"rotateIntake_Servo");
        linearSlide_Servo = hardwareMap.get(Servo.class,"linearSlide_Servo");
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

        /// ALL CODE RUNNING IN HERE
        while(opModeIsActive()) {

            // Drive normally
            driveTrain(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x);

            // This determines Which state it is in and runs the correct one
            switch (state)
            {
                // If the state is Intake, run
                case INTAKE:

                    // Move the Intake Servos based on triggers
                    twoIntake_Servo_Power = gamepad2.right_trigger - gamepad2.left_trigger;

                    // Move the Whole Intake Assembly to Intake or Transfer to Outtake
                    if (gamepad2.dpad_up)
                        rotateIntake_Servo_Position = 1; // Transfer Outtake
                    else if (gamepad2.dpad_down)
                        rotateIntake_Servo_Position = 0; // Intake

                    // Linear Slide Position based on stick
                    linearSlide_Servo_Position += gamepad2.left_stick_y * 0.1; // 0.1-0.7 (Extended - Collapsed)

                    // Make Sure its Withen The Parameters
                    if (linearSlide_Servo_Position > 0.7)
                        linearSlide_Servo_Position = 0.7;
                    else if (linearSlide_Servo_Position < 0.1)
                        linearSlide_Servo_Position = 0.1;

                    // Outtake Sequence
                    // Move the Sample near the Outtake
                    if (gamepad2.y)
                    {
                        linearSlide_Servo_Position = 0.5; // CHECK IDK
                        rotateIntake_Servo_Position = 1;
                        timer(0, 0);
                    }
                    // Transfer the Sample to the Outtake
                    if (timer(300, 0))
                    {
                        twoIntake_Servo_Power = 0.5;
                        timer(0, 0);
                    }
                    // Raise the Linear Slide to Outtake
                    else if (timer(200, 0))
                    {
                        linearSlide_Motor_Position = 1000; // IDK CHECK
                        timer(0, 0);
                    }
                    // Reset Intake Components
                    else if (timer(300, 0))
                    {
                        linearSlide_Servo_Position = 0.7;
                        rotateIntake_Servo_Position = 0;
                        timer(0, 0);
                        state = State.OUTTAKE;
                    }

                // If the state is Outtake, run
                case OUTTAKE:

                    // Move to outtake
                    if (gamepad1.right_bumper) {
                        outtake_Servo_Position = 0.9;
                        timer(0, 1);
                    }
                    // Wait, then return to Intake
                    if (timer(800, 1)) {
                        outtake_Servo_Position = 0.4;
                        linearSlide_Motor_Position = 100; // IDK CHECK
                        timer(0, 1);
                        state = State.INTAKE;
                    }

            }// Switch (state) end

            // Move the Motors & Servos (Define How move in upper code then move here so less code required)
            linearSlide_Motor.setTargetPosition(linearSlide_Motor_Position);
            linearSlide_Motor.setVelocity(100);

            leftIntake_Servo.setPower(twoIntake_Servo_Power);
            rightIntake_Servo.setPower(-twoIntake_Servo_Power);

            rotateIntake_Servo.setPosition(rotateIntake_Servo_Position);
            linearSlide_Servo.setPosition(linearSlide_Servo_Position);
            outtake_Servo.setPosition(outtake_Servo_Position);

            updateTelemetry();

        } // while opModeIsActive end

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
    }// timera end

    // Telemetry method
    public void updateTelemetry() {
        telemetry.addData("State: ", state);
        telemetry.addData("RunTime: ", runtime);
        telemetry.addLine();
        telemetry.addData("leftTop_Motor Power: ", leftTop_Motor.getPower());
        telemetry.addData("leftTop_Motor Velocity: ", leftTop_Motor.getVelocity());
        telemetry.addData("leftTop_Motor Current Position: ", leftTop_Motor.getCurrentPosition());
        telemetry.addLine();
        telemetry.addData("leftBot_Motor Power: ", leftBot_Motor.getPower());
        telemetry.addData("leftBot_Motor Velocity: ", leftBot_Motor.getVelocity());
        telemetry.addData("leftBot_Motor Current Position: ", leftBot_Motor.getCurrentPosition());
        telemetry.addLine();
        telemetry.addData("rightTop_Motor Power: ", rightTop_Motor.getPower());
        telemetry.addData("rightTop_Motor Velocity: ", rightTop_Motor.getVelocity());
        telemetry.addData("rightTop_Motor Current Position: ", rightTop_Motor.getCurrentPosition());
        telemetry.addLine();
        telemetry.addData("rightBot_Motor Power: ", rightBot_Motor.getPower());
        telemetry.addData("rightBot_Motor Velocity: ", rightBot_Motor.getVelocity());
        telemetry.addData("rightBot_Motor Current Position: ", rightBot_Motor.getCurrentPosition());
        telemetry.addLine();
        telemetry.addData("linearSlide_Motor Velocity: ", linearSlide_Motor.getVelocity());
        telemetry.addData("linearSlide_Motor Current Position: ", linearSlide_Motor.getCurrentPosition());
        telemetry.addData("linearSlide_Motor_Position: ", linearSlide_Motor_Position);
        telemetry.addLine();
        telemetry.addData("leftIntake_Servo: ", leftIntake_Servo.getPower());
        telemetry.addData("rightIntake_Servo: ", rightIntake_Servo.getPower());
        telemetry.addLine();
        telemetry.addData("rotateIntake_Servo: ", rotateIntake_Servo.getPosition());
        telemetry.addData("rotateIntake_Servo_Position: ", rotateIntake_Servo_Position);
        telemetry.addLine();
        telemetry.addData("linearSlide_Servo: ", linearSlide_Servo.getPosition());
        telemetry.addData("linearSlide_Servo_Position: ", linearSlide_Servo_Position);
        telemetry.addLine();
        telemetry.addData("outtake_Servo: ", outtake_Servo.getPosition());
        telemetry.addData("outtake_Servo_Position: ", outtake_Servo_Position);
        telemetry.addLine();
        telemetry.addData("Red: ", color_Sensor.red());
        telemetry.addData("Green: ", color_Sensor.green());
        telemetry.addData("Blue: ", color_Sensor.blue());
        telemetry.addLine();
        telemetry.update();
    }//update telemetry end

}// class end
