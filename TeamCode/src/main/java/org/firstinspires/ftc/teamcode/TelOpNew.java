/*
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "TeleOpStateNew", group = "AA")
public class TelOpNew extends LinearOpMode {
    BaseClass rbg;// hardware init at Mecanumdrive.
    double speed_factor = 1.0;

//    double[] deltaSlidePowers= new double[1000];
//
//    double deltaPower = 0;

//    int index = 0;



    public enum State {
        IDLE,
        SAMPLEINTAKE,
        INTAKEIDLE,
        SPECINTAKE,
        SAMPLELIFT,
        SAMPLEOUTTAKE,
        SPECOUTTAKE,
        HANG

    }

    State state = State.IDLE;
    @Override
    public void runOpMode() {
        Pose2d p1 = new Pose2d(0, 0, 0);
        rbg = new BaseClass(this, p1);
        rbg.newlinearslides=true;
        if (isStopRequested()) return;
        telemetry.addLine("Wait ! Initializing............. ");
        telemetry.addLine("Do not press start  ");
        telemetry.update();
        rbg.init(0);
        rbg.init(2);
        sleep(500);
        telemetry.addLine("Press Start Now!:");
        telemetry.update();
        rbg.pidf_index=rbg.pidf_idle;



//        rbg.Front_led.setPosition(0.5);
//        rbg.Back_led.setPosition(0.5);
        waitForStart();
      //  rbg.pp0=new Pose2d(0, 0, 0);
        rbg.timer(0,rbg.start);
        rbg.pre_idle();
        while (opModeIsActive()) {
            switch (state) {
               case IDLE:
                   if(!rbg.flag[rbg.idleready])
                   {
                       rbg.idle_ready();
                       if(speed_factor<1)speed_factor=rbg.speed_index;
                       break;
                   }
                   if(gamepad2.right_bumper) {
                       rbg.pidf_index=rbg.pidf_sampleintake;
                       rbg.pre_sampleintake();
                       state = State.SAMPLEINTAKE;
                       break;
                   }
                   if(gamepad2.left_bumper) {
                       rbg.pidf_index=rbg.pidf_idle_specin;
                       rbg.pre_specintake(false);
                       state = State.SPECINTAKE;
                       break;
                   }
                   break;

case SAMPLEINTAKE:
                   if(!rbg.flag[rbg.sampleintakeready])
                   {
                       speed_factor=0.4;
                       rbg.smoothshift = true;
                       rbg.pidf_index=rbg.pidf_sampleintake;
                       rbg.sampleintake_ready(gamepad2.right_bumper);
                       break;
                   }

                   if(gamepad2.right_bumper&&(rbg.drivinginput<0.1)) {
                       rbg.pidf_index=rbg.pidf_sampleintake;
                       if(rbg.sampleintake())// if wrong color or nothing,
                       {
                           speed_factor = 1;
                           state = State.INTAKEIDLE;
                           break;
                       }
                   }

                   if(gamepad2.left_bumper) {

                       rbg.pre_specintake(false);
                       rbg.pidf_index=rbg.pidf_idle_specin;
                       speed_factor=1;
                       state = State.SPECINTAKE;

                       break;
                   }

               

                  rbg.intake_smooth_shift2(gamepad2.right_stick_y);
                  rbg.intake_claw_rotate(gamepad2.left_stick_x);
                  break;

  case INTAKEIDLE:
//                    if(!rbg.timer(1000,rbg.intake)&&gamepad2.right_bumper){
//                       rbg.resampleintake();
//                        state = State.SAMPLEINTAKE;
//                        break;
//                    }

                    if(!rbg.flag[rbg.intakeidleready])
                    {
                        rbg.intakeidle_ready();
                        break;
                    }
                    if(gamepad2.right_bumper&&rbg.timer(500, rbg.intake)) {
                        rbg.pre_samplelift(false);
                        state = State.SAMPLELIFT;

                        break;
                    }

                    if(gamepad2.left_bumper) {
                        rbg.pidf_index=rbg.pidf_idle_specin;
                        rbg.pre_specintake(false);
                      state = State.SPECINTAKE;
                        break;
                    }


                    if(gamepad1.touchpad) {
                        rbg.pidf_index=rbg.pidf_idle_specin;
                        rbg.pre_specintake(true);
                        speed_factor=0.4;
                        state = State.SPECINTAKE;
                        break;
                    }


                    if(gamepad1.right_bumper) {
                        if(rbg.pre_samplelift(true))  state = State.SAMPLELIFT;
                        break;
                    }

                break;

  case SPECINTAKE:
                   if(!rbg.flag[rbg.specintakeready])
                   {
                       rbg.specintake_ready();
                       break;
                   }
                   if(gamepad2.right_bumper&&rbg.flag[rbg.claw_lock]) {
                       rbg.linearslide(rbg.slide_idle,rbg.slidev2 );
                       rbg.pidf_index=rbg.pidf_specin_sampleout;
                       rbg.pre_samplelift(false);
                       state = State.SAMPLELIFT;
                       speed_factor=1.0;
                       break;
                   }

                   rbg.inspec_handleadj(gamepad2.right_stick_y);



                   if(gamepad1.touchpad||rbg.flag[rbg.placement]){
                       rbg.specplacment();
                       speed_factor=0.4;
                   }
                   if(gamepad1.right_bumper) {
                       rbg.autospec_intake();

                       if(gamepad1.right_bumper) {

                           rbg.specmove();
                           rbg.pre_specouttake();
                           state = State.SPECOUTTAKE;
                           speed_factor=1;
                           break;
                       }

                       rbg.pre_specouttake();
                       speed_factor=1;
                       state = State.SPECOUTTAKE;
                       break;
                   }

                if(gamepad1.right_trigger>0.6)
                    {
                        rbg.specintake();
                        if(gamepad1.right_trigger>0.6)
                        {
                        rbg.specmove();
                        state = State.SPECOUTTAKE;
                        speed_factor=1;
                         break;
                        }

                    rbg.pre_specouttake();
                    speed_factor=1;
                    state = State.SPECOUTTAKE;
                     break;
                }

                break;

  case SAMPLELIFT:
                   if(!rbg.flag[rbg.sampleliftready]) {
                      if(!rbg.flag[rbg.lift] && gamepad1.right_bumper)  rbg.flag[rbg.lift]=true;
                      rbg.samplelift_ready();
                       break;
                   }
                if(gamepad1.right_bumper||rbg.flag[rbg.lift]) {
                   rbg.pre_sampleouttake();
                   speed_factor=0.3;
                   state = State.SAMPLEOUTTAKE;
                   break;
                }
                if(gamepad2.left_bumper) {
                    rbg.pidf_index=rbg.pidf_sampleout_specin;
                    rbg.pre_specintake(false);
                    state = State.SPECINTAKE;
                    break;
                    }
               break;


 case SAMPLEOUTTAKE:

                if(!rbg.flag[rbg.sampleouttakeready]) {
                    rbg.sampleouttake_ready();
                    break;
                }
                 if(gamepad1.right_bumper) {
                     rbg.sampleouttake();
                     rbg.pidf_index=rbg.pidf_sampleout_idle;
                     rbg.pre_idle();
                     state = State.IDLE;
                     break;
                 }

                 break;


  case SPECOUTTAKE:
                   if(!rbg.flag[rbg.specouttakeready])
                   {
                       rbg.specouttake_ready();
                       break;
                   }

                   rbg.outspec_handleadj(gamepad2.right_stick_y);

                   if(gamepad1.right_bumper) {
                       rbg.specouttake();
                       rbg.pre_idle();
                       state = State.IDLE;
                       break;

                   }

                   break;



    case HANG:


                break;

        }



 rbg.armrotatePIDF();




            if(gamepad1.left_bumper && !rbg.flag[rbg.hang] && !rbg.flag[rbg.hang0]) {
               if (rbg.drop())  state = State.IDLE;
             else state = State.SAMPLEINTAKE;
            }

            if (rbg.timer(88000, rbg.start) || rbg.flag[rbg.force]) {

                if (gamepad2.share || rbg.flag[rbg.hang]) {
                    state= State.HANG;
                    speed_factor = 1.0;
                    rbg.pre_hang2();
                }
                if (gamepad1.share) {
                   rbg.hang2(gamepad1.right_bumper);
                }

             if(!rbg.flag[rbg.vb]&&!rbg.flag[rbg.hang]&&rbg.timer(100000,rbg.start)){
                    gamepad2.rumble(0.9,0.9,500);
                    gamepad1.rumble(0.9,0.9,500);

                    rbg.flag[rbg.vb]=true;
                }


            }

            if (gamepad2.ps) rbg.flag[rbg.force] = true;
            if(gamepad1.ps)   {
                rbg.linersliderest();
                state= State.IDLE;


            }

            rbg.robot_centric(gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, speed_factor);

            telemetry.addData("State",state);
            telemetry.addData("Slide encoder",rbg.revEncoder.getCurrentPosition());

            telemetry.addData("Slide pos",rbg.slidePos);
//            telemetry.addData("Arm pos", rbg.arm_angle);
            telemetry.addData("Slide powers", rbg.lpower);
            telemetry.update();
//
//            if (rbg.Slide_top.getCurrent(CurrentUnit.AMPS) > 8 || rbg.Slide_bot.getCurrent(CurrentUnit.AMPS) > 8){
//                gamepad2.rumble(0.9,0.9,500);
//                gamepad1.rumble(0.9,0.9,500);
//
//            }



//            telemetry.addData("Motor encoder", rbg.Slide_top.getCurrentPosition());
//            telemetry.addData("External encoder", rbg.revEncoder.getCurrentPosition());
//
//            telemetry.addData("total power" , rbg.lpower);
//            telemetry.addData("PID Power", rbg.lpid);
//            telemetry.addData("Rev target", rbg.revtarget);
//            telemetry.update();






         //    telemetry.addData("armlinerslide top", rbg.Slide_top.getCurrentPosition());
        //    telemetry.addData("armlinerslide bot", rbg.Slide_bot.getCurrentPosition());
////
//            telemetry.addData("red", rbg.Claw_color.red());
//            telemetry.addData("green", rbg.Claw_color.green());
//            telemetry.addData("blue", rbg.Claw_color.blue());
//            telemetry.addData("color", rbg.Claw_color.alpha());
////
////            telemetry.addData("Arm angle", rbg.arm_angle);
////
////            rbg.updatePoseEstimate();
////
////            telemetry.addData("Pos x" , rbg.pose.position.x);
////            telemetry.addData("Pos y" , rbg.pose.position.y);
////            telemetry.addData("imu",rbg.imu.getRobotYawPitchRollAngles().getYaw((AngleUnit.DEGREES)));
//////
////            telemetry.addData("Arm right power", rbg.power);
////
////            telemetry.addData("Speed factor", speed_factor);
//
////            telemetry.addData("Preidle",rbg.flag[rbg.preidle]);
////            telemetry.addData("Idleready",rbg.flag[rbg.idleready]);
//////
////           telemetry.addData("gerbox", rbg.Gearbox.getPosition());
////            telemetry.addData("K value", rbg.k);
////
////            telemetry.addData("intake_lvel", rbg.intake_level);
////            telemetry.addData("Right arm motor current", rbg.Arm_right.getCurrent(CurrentUnit.AMPS));
////            telemetry.addData("Left arm motor current", rbg.Arm_left.getCurrent(CurrentUnit.AMPS));
////
////            telemetry.addData("Top slide motor current", rbg.Slide_top.getCurrent(CurrentUnit.AMPS));
////            telemetry.addData("Bot slide motor current", rbg.Slide_bot.getCurrent(CurrentUnit.AMPS));
////            telemetry.addData("Bar dist", rbg.bar_dist.getDistance(DistanceUnit.MM));

//            telemetry.addData("LeftHanlde ", rbg.Left_handle.getPosition());
//            telemetry.addData("Right handle ", rbg.Right_handle.getPosition());
//            telemetry.addData("LeftHanlde current  ", rbg.curleft_handle);
//            telemetry.addData("Right handle current ", rbg.curright_handle);
//            telemetry.addData("Flag specouttakje ready",rbg.flag[rbg.specouttakeready]);
//
//            telemetry.addData("Arm angle",rbg.arm_angle);
//            telemetry.update();


//            deltaPower = Math.abs(rbg.Slide_top.getPower() - rbg.Slide_bot.getPower());
//
//            telemetry.addData("Motor delta power", deltaPower);
//            telemetry.addData("Top Power",rbg.Slide_top.getPower());
//            telemetry.addData("Bot power", rbg.Slide_bot.getPower());
//            telemetry.update();

            }
        }
    }

 */