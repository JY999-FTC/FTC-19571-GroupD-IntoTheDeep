package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class AutoBasket2024 extends DriveMethods {

    double stateStartTime = -1;

    enum State {
        Finished,
        Unstarted,
        RaiseArm,
        StrafeRight,
        ExtendSlider,
        OpenClaw
    }
    State currentState = State.Unstarted;

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("code", "running");
        telemetry.addData("time", "%.1f", getRuntime());
        telemetry.addData("sliderLength","%.1f", (double) robot.sliderMotor.getCurrentPosition());
        telemetry.addData("imu", "%.1f", robot.imu.getRobotYawPitchRollAngles().getYaw());

        telemetry.addData("state", currentState);
        switch (currentState) {
            case Unstarted:
                changeState(State.StrafeRight);
                break;
            case StrafeRight:
                omniDrive(0, 0.25, 0);

                if (getStateTime() >= 1.5) {
                    omniDrive(0, 0, 0);

                    changeState(State.RaiseArm);
                }
                break;
            case RaiseArm:
                robot.wormGear.setPower(0.5);

                if (robot.wormGearAngle() >= 68.91) {
                    robot.wormGear.setPower(0);

                    changeState(State.ExtendSlider);
                }
                break;
            case ExtendSlider:
                robot.sliderMotor.setPower(1);
                robot.sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                setSliderAndReturnConstraint(robot.MAX_HORIZONTAL_SLIDER_TICKS);

                if (robot.sliderMotor.getCurrentPosition() >= 2200) {
                    changeState(State.OpenClaw);
                }
                break;
            case OpenClaw:
                robot.clawServo.setPosition(robot.CLAW_OPEN);
                changeState(State.Finished);
                break;
            case Finished:
                omniDrive(0, 0, 0);
                break;
        }

    }
    void changeState(State nextState) {
        currentState = nextState;
        stateStartTime = getRuntime();
    }

    double getStateTime() {
       return getRuntime() - stateStartTime;
    }


//        if (getRuntime() < 2) {
//            omniDrive(0, 1, 0);
//        } else if (getRuntime() < 4) {
//            omniDrive(0, 0, 1);
//        }
//        else {
//                omniDrive(0, 0, 0);
//            }
//        }

    //            case MoveForward:
//                omniDrive(0.5, 0, 0);
//
//                if (getStateTime() >= 0.7) {
//                    omniDrive(0, 0, 0);
//
//                    changeState(State.RaiseArm);
//                }
//                break;
    }