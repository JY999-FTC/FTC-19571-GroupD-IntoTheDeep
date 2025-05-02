package org.firstinspires.ftc.teamcode;

public class GroupD_OpMode_V1 extends GroupD_BasicDrive {


    public void runOpMode() throws InterruptedException{

        driveTrain(gamepad1.right_stick_x, gamepad1.right_stick_y, -gamepad1.left_stick_x);
        updateTelemetry();

    } // OpMode end


} // class end
