package org.firstinspires.ftc.teamcode.JackBurr.Servos;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class AxonPickupTest2 extends OpMode {
    public Servo grippers;
    public Servo left_servo;
    public AnalogInput left_servo_encoder;
    public AnalogInput right_servo_encoder;
    public Servo right_servo;
    public ElapsedTime button_timer = new ElapsedTime();
    public ElapsedTime servoTimer = new ElapsedTime();
    public enum ElbowState {
        DOWN,
        UP
    }
    public enum GripperState {
        OPEN,
        CLOSED
    }
    public ElbowState elbowState = ElbowState.DOWN;
    public GripperState gripperState = GripperState.OPEN;
    public int GRIPPERS_CLOSED = 1;
    public int GRIPPERS_OPEN = 0;
    public double LEFT_SERVO_ZERO = 60.9;
    //TODO: REPLACE THESE VALUES WITH CORRECT VALUES
    public double RIGHT_SERVO_ZERO = 0.0;

    @Override
    public void init() {
        grippers = hardwareMap.get(Servo.class, "grippers");
        left_servo = hardwareMap.get(Servo.class, "left_diff");
        right_servo = hardwareMap.get(Servo.class, "right_diff");
        left_servo_encoder = hardwareMap.get(AnalogInput.class,"left_servo_encoder");
        right_servo_encoder = hardwareMap.get(AnalogInput.class,"right_servo_encoder");
        left_servo.setDirection(Servo.Direction.REVERSE);
        right_servo.setDirection(Servo.Direction.REVERSE);
        left_servo.setPosition(1);
        right_servo.setPosition(1);
        grippers.setPosition(1);
    }

    @Override
    public void loop() {
        if(gamepad1.x && button_timer.seconds() > 0.3){
            switch (elbowState){
                case UP:
                    elbowState = ElbowState.DOWN;
                    break;
                case DOWN:
                    servoTimer.reset();
                    while (servoTimer.seconds() < 2) {
                        grippers.setPosition(GRIPPERS_CLOSED);
                        gripperState = GripperState.CLOSED;
                    }
                    elbowState = ElbowState.UP;
            }
            button_timer.reset();
        }
        if(gripperState == GripperState.OPEN){
            grippers.setPosition(GRIPPERS_OPEN);
        }
        else {
            grippers.setPosition(GRIPPERS_CLOSED);
        }
        if(elbowState == ElbowState.UP){
            if(grippers.getPosition() == 1) {
                run_left_servo_to_zero();
                run_right_servo_to_zero();
            }
        }
        else {
            left_servo.setPosition(1);
            right_servo.setPosition(1);
        }
        telemetry.addData("Left Servo Position:",left_servo.getPosition());
        telemetry.addData("Right Servo Position:",right_servo.getPosition());
        telemetry.addData("Left Encoder: ", String.valueOf(left_servo_encoder.getVoltage() / 3.3 * 360) + "°");
        telemetry.addData("Right Encoder: ", String.valueOf(right_servo_encoder.getVoltage() / 3.3 * 360) + "°");
        telemetry.addData("Servo position:", grippers.getPosition());
    }
    public void run_left_servo_to_zero(){
        double encoder_pos = left_servo_encoder.getVoltage() / 3.3 * 360;
        if (!is_in_range(encoder_pos, LEFT_SERVO_ZERO - 2, LEFT_SERVO_ZERO + 2)){
            if (encoder_pos < LEFT_SERVO_ZERO - 2){
                double target = left_servo.getPosition() - 0.05;
                left_servo.setPosition(target);
                telemetry.addData("Left Servo is left of zero. Moving from " + left_servo.getPosition() + " to: ", target);
            }
            else if (encoder_pos > LEFT_SERVO_ZERO + 2){
                double target = left_servo.getPosition() + 0.05;
                left_servo.setPosition(target);
                telemetry.addData("Left Servo is right of zero. Moving from " + left_servo.getPosition() + " to: ", target);
            }
        }
    }

    public void run_right_servo_to_zero(){
        double encoder_pos = right_servo_encoder.getVoltage() / 3.3 * 360;
        if (!is_in_range(encoder_pos, RIGHT_SERVO_ZERO - 2, RIGHT_SERVO_ZERO + 2)){
            if (encoder_pos < RIGHT_SERVO_ZERO - 2){
                double target = right_servo.getPosition() - 0.05;
                right_servo.setPosition(target);
                telemetry.addData("Left Servo is left of zero. Moving from " + right_servo.getPosition() + " to: ", target);
            }
            else if (encoder_pos > RIGHT_SERVO_ZERO + 2){
                double target = right_servo.getPosition() + 0.05;
                right_servo.setPosition(target);
                telemetry.addData("Left Servo is right of zero. Moving from " + right_servo.getPosition() + " to: ", target);
            }
        }
    }

    public boolean is_in_range(double number, double left_number, double right_number){
        if(number > left_number && number < right_number) {
            return true;
        }
        else if(number == left_number || number == right_number) {
            return true;
        }
        else {
            return false;
        }
    }
}
