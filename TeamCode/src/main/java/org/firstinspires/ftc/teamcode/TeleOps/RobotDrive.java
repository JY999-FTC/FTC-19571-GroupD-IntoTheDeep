package org.firstinspires.ftc.teamcode.TeleOps;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.BACK;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.START;

import android.annotation.SuppressLint;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.State;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class RobotDrive {

    private final GamepadEx gamepad_1;
    private final GamepadEx gamepad_2;
    private final RobotHardware robot;
    private ControlMode controlMode = ControlMode.ROBOT_CENTRIC;

    private ElapsedTime debounceTimer = new ElapsedTime(); // Timer for debouncing


    private boolean startPressed = false;
    private boolean backPressed = false;

    private double powerFactor;

    public RobotDrive(RobotHardware robot, GamepadEx gamepad_1, GamepadEx gamepad_2) {
        this.robot = robot;
        this.gamepad_1 = gamepad_1;
        this.gamepad_2 = gamepad_2;
    }

    public void Init() {
        // Initialize IMU from RobotHardware
        robot.initIMU();
    }

    @SuppressLint("DefaultLocale")
    public void DriveLoop() {
        // Toggle control mode
        if ((gamepad_1.getButton(START) || gamepad_2.getButton(START)) && !startPressed) {
            toggleControlMode();
            debounceTimer.reset();
            startPressed = true;
        } else if (!gamepad_1.getButton(START) || !gamepad_2.getButton(START)) {
            startPressed = false;
        }

        // Reset IMU heading using button back and reset odometry
        if (gamepad_1.getButton(BACK) || gamepad_2.getButton(BACK) && !backPressed) {
            robot.initIMU();
            //robot.resetDriveEncoders();
            debounceTimer.reset();
            backPressed = true;
        } else if (!gamepad_1.getButton(BACK) || !gamepad_2.getButton(BACK)) {
            backPressed = false;
        }

        if(gamepad_1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>0.5 || gamepad_2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>0.5){
            powerFactor = RobotActionConfig.powerFactor / 2;
        }
        else {
            powerFactor = RobotActionConfig.powerFactor;
        }
        double drive = 0.0;
        double strafe = 0.0;
        double rotate = 0.0;
        if (Math.abs(gamepad_1.getRightY()) > 0.1 || Math.abs(gamepad_1.getRightX()) > 0.1 || Math.abs(gamepad_1.getLeftX()) > 0.1) {
            drive = -gamepad_1.getRightY();
            strafe = gamepad_1.getRightX();
            rotate = gamepad_1.getLeftX();
        } else if (Math.abs(gamepad_2.getRightY()) > 0.1 || Math.abs(gamepad_2.getRightX()) > 0.1 || Math.abs(gamepad_2.getLeftX()) > 0.1) {
            drive = -gamepad_2.getRightY();
            strafe = gamepad_2.getRightX();
            rotate = gamepad_2.getLeftX();
        }
        /**
        // Set gamepad joystick power
        double drive = 0.0;
        if (Math.abs(gamepad_1.getRightY()) > 0.1) {
            // Use Gamepad 1 if there's input
            drive = -gamepad_1.getRightY();
        } else if (Math.abs(gamepad_2.getRightY()) > 0.1) {
            // If Gamepad 1 has no input, use Gamepad 2
            drive = -gamepad_2.getRightY();
        }

        double strafe = 0.0;
        if (Math.abs(gamepad_1.getRightX()) > 0.1) {
            strafe = gamepad_1.getRightX();
        } else if (Math.abs(gamepad_2.getRightX()) > 0.1) {
            strafe = gamepad_2.getRightX();
        }

        double rotate = 0.0;
        if (Math.abs(gamepad_1.getLeftX()) > 0.1) {
            rotate = gamepad_1.getLeftX();
        } else if (Math.abs(gamepad_2.getLeftX()) > 0.1)
            rotate = gamepad_2.getLeftX();
         **/

        // Get robot's current heading
        double currentHeading = getRobotHeading();

        // Mecanum drive calculations
        setMecanumDrivePower(drive, strafe, rotate, currentHeading, powerFactor);

        // Update telemetry with the latest data
        // empty
    }// end of driveloop


    private double getRobotHeading() {
        // Get the robot's heading from IMU
        // double heading = robot.imu().firstAngle;
        double heading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        while (heading > 180.0) {
            heading -= 360.0;
        }
        while (heading < -180.0) {
            heading += 360.0;
        }
        return -heading;
    }

    private void toggleControlMode() {
        if (controlMode == ControlMode.FIELD_CENTRIC) {
            controlMode = ControlMode.ROBOT_CENTRIC;
        } else {
            controlMode = ControlMode.FIELD_CENTRIC;
        }
    }

    private void setMecanumDrivePower(double drive, double strafe, double rotate, double currentHeading, double powerFactor) {
        // Determine the drive mode
        if (controlMode == ControlMode.FIELD_CENTRIC) {
            // Adjust for field-centric control using the gyro angle
            double headingRad = Math.toRadians(currentHeading);
            double temp = drive * Math.cos(headingRad) + strafe * Math.sin(headingRad);
            strafe = -drive * Math.sin(headingRad) + strafe * Math.cos(headingRad);
            drive = temp;
        }

        // Mecanum wheel drive formula
        double frontLeftPower = drive + strafe + rotate;
        double frontRightPower = drive - strafe - rotate;
        double backLeftPower = drive - strafe + rotate;
        double backRightPower = drive + strafe - rotate;

        // Constrain the power within +-1.0
        double maxPower = Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backRightPower), Math.abs(backLeftPower))
        );

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backRightPower /= maxPower;
            backLeftPower /= maxPower;
        }



        // Set motor powers
        robot.frontLeftMotor.setPower(Range.clip(frontLeftPower * powerFactor, -1.0, 1.0));
        robot.frontRightMotor.setPower(Range.clip(frontRightPower * powerFactor, -1.0, 1.0));
        robot.backLeftMotor.setPower(Range.clip(backLeftPower * powerFactor, -1.0, 1.0));
        robot.backRightMotor.setPower(Range.clip(backRightPower * powerFactor, -1.0, 1.0));

    }

    // Method to get left encoder count
    /*
    public int [] getEncoderCounts() {
        int[] counts = new int[3];
        counts[0] = robot.leftodometry.getCurrentPosition();
        counts[1] = robot.rightodometry.getCurrentPosition();
        counts[2] = robot.centerodometry.getCurrentPosition();
        return counts;
    }
    */
    public double[] getVelocity() {
        double[] velocities = new double[4];
        velocities[0] = robot.frontLeftMotor.getVelocity();
        velocities[1] = robot.frontRightMotor.getVelocity();
        velocities[2] = robot.backLeftMotor.getVelocity();
        velocities[3] = robot.backRightMotor.getVelocity();
        return velocities;
    }

    public enum ControlMode {
        FIELD_CENTRIC,
        ROBOT_CENTRIC
    }
    public ControlMode getControlMode() {
        return controlMode;
    }
}
