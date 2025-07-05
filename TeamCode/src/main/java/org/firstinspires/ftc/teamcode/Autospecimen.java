package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.arcrobotics.ftclib.util.InterpLUT;

@Autonomous(name = "AutoSpecimen State", group = "A")
//@Disabled
@Config
public class Autospecimen extends LinearOpMode {
    // private ElapsedTime runtime = new ElapsedTime();


    Pose2d pp = new Pose2d(0, 0, 0);
    BaseClass rbg;


    @Override
    public void runOpMode() {
        rbg = new BaseClass(this, new Pose2d(0,0,0));
        telemetry.addLine("Make sure linearslide was fully in before start and do not move the robot anymore;");
        telemetry.update();
         rbg.newlinearslides=true;
        rbg.init(0);
        sleep(500);
        rbg.init(1);
        rbg.deadzonecontrol=true;
        telemetry.addLine("Load the specimen in the claw and press Driver right bumper;");
        telemetry.update();
        while(!isStopRequested())
        {
            sleep(20);
            if(gamepad2.dpad_up&&rbg.courntnumber<4) {
                rbg.courntnumber = rbg.courntnumber + 1;
                telemetry.addData("Field location Number",rbg.courntnumber);
                telemetry.update();

            }
            if(gamepad2.dpad_down&&rbg.courntnumber>0) {
                rbg.courntnumber = rbg.courntnumber - 1;
                telemetry.addData("Field location Number",rbg.courntnumber);
                telemetry.update();
            }


            if(gamepad1.right_bumper){
             rbg.Claw.setPosition(rbg.claw_close);
             break;
            }
        }
        rbg.autocourtadjustment();
        sleep(200);
        rbg.updatePoseEstimate();
        telemetry.addData("Field location Number",rbg.courntnumber);
        telemetry.addLine("1-- First Filed Red side, 2--First Field Blue side");
        telemetry.addLine("1-- First Filed Red side, 2--First Field Blue side");
        telemetry.addData(">", "Press Play to start op mode ");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {

//            rbg.delay(4000);

            rbg.aspec_outtake();
            rbg.afmove(0,true,true);//strafe samples


            rbg.afmove(1,false,false);// forward for sampels


            rbg.afmove(2,true,false);//strafe for first samples
            rbg.afmove(3,false,false);//push the first sample

            rbg.afmove(4,false,false);//forward for second sample
            rbg.afmove(5,true,false);//strafe for second sampl
            rbg.afmove(6,false,false);//push the seond samples and intake
            rbg.afmove(20,false,false);// forward for 3rd
            rbg.afmove(21,true,false);//strafe  for 3rd
            rbg.afmove(22,false,false);// push for 3 rd
            rbg.stop_drive();
          //  rbg.delay(10000000);
            rbg.aspec_intake();
            rbg.afmove(7,true,true);//strafe for outtake

//            rbg.delay(100000);
            rbg.aspec_outtake();;
            rbg.afmove(8,true,true);//strafe for intake
//            rbg.delay(200000000);
            rbg.aspec_intake();;
            rbg.afmove(9,true,true);//strafe for outtake
            rbg.aspec_outtake();;
            rbg.afmove(10,true,true);//strafe for i ntake;
            rbg.aspec_intake();
            rbg.afmove(11,true,true);//strafe for outtake;
            rbg.aspec_outtake();
            rbg.afmove(12,true,true);//strafe for park;
            rbg.aspec_intake();
            rbg.afmove(13,true,true);//strafe for park;
            rbg.flag[rbg.last]=true;
            rbg.aspec_outtake();
            rbg.afmove(14,true,true);//strafe for park;
//            rbg.pidf_index=rbg.pidf_sampleout_idle;
//            rbg.pidfsetting();
//            rbg.armRotateLeft.setPower(0);
//            rbg.armRotate.setPower(0);
            //  rbg.armRotateLeft.setPower((0));
            rbg.delay(50000);


        }
    }
}