package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "AutoSample State", group = "B")
//@Disabled
@Config
public class Autosample extends LinearOpMode {
    // private ElapsedTime runtime = new ElapsedTime();


    Pose2d pp = new Pose2d(0, 0, 0);
    BaseClass rbg;
    // Pose2d start = new Pose2d(0, 0, Math.toRadians(0));


    @Override
    public void runOpMode() {
        rbg = new BaseClass(this, pp);
        rbg.init(0);

        telemetry.addLine("Make sure linearslide was fully in befroe start and do not move the robot anymore;");
        telemetry.update();
        // rbg.init(0);
        sleep(200);
        rbg.init(3);
        telemetry.addLine("Preload the specimen in the claw and press Driver right bumper;");
        telemetry.update();
        while (!isStopRequested()) {
            sleep(20);
            if (gamepad1.right_bumper) {
                rbg.Claw.setPosition(rbg.claw_close);
                break;
            }
        }

            sleep(200);
            rbg.updatePoseEstimate();
            telemetry.addData("x", rbg.pose.position.x);
            telemetry.addData("y", rbg.pose.position.y);
            telemetry.addData(">", "Press Play to start op mode ");
            telemetry.update();
            waitForStart();


            // rbg.timer(0,8);
            while (opModeIsActive()) {
                rbg.asamplefirstmove();
                rbg.amove(0, false); //preload sample move to outake
                rbg.asample_outtake();
                rbg.amove(1, true);// forward to 1st sample for intake;
                rbg.asample_intake();
                rbg.amove(2, false);// first sample outtake
                rbg.asample_outtake();
                rbg.amove(3, true);//second sample intake
                rbg.asample_intake();
                rbg.amove(4, false);//secone smaple outatek
                rbg.asample_outtake();
                rbg.amove(5, true);//third sampe intake
                rbg.flag[rbg.last] = true;
                rbg.asample_intake();
                rbg.amove(6, false);
                rbg.asample_outtake();
                rbg.delay(500);
                rbg.stop_drive();
                sleep(1000000);
                //  rbg.pidfsetting(rbg.arm_angle_preintake - 3);
                // telemetry.addData("red", rbg.Claw_color.getConnectionInfo());
//                telemetry.addData("green", rbg.Claw_color.green());
//                telemetry.addData("blue", rbg.Claw_color.blue());
//                telemetry.addData("color", rbg.Claw_color.alpha());
//                telemetry.addData("x0", rbg.xo);
//                telemetry.addData("y0", rbg.yo);
//                telemetry.addData("a0", rbg.ao);
//                telemetry.addData("x", rbg.pose.position.x);
//                telemetry.addData("y", rbg.pose.position.y);
//                telemetry.addData("angle", rbg.imu.getRobotYawPitchRollAngles().getYaw((AngleUnit.DEGREES)));
//                telemetry.update();
                // rbg.delay(1000000000);

            }
        }
    }
