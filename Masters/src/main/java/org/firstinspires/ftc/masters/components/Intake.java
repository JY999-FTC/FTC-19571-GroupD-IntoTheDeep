package org.firstinspires.ftc.masters.components;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Intake {
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    PIDController pidController;

    public static double p = 0.0015, i = 0, d = 0;
    public static double f = 0.05;

    public static double ticks_in_degrees = (double) 8192 / 360;

    Init init;

    Telemetry telemetry;
    Servo intakeLeft;
    Servo intakeRight;
    DcMotor intakeMotor;
    DcMotor extendo;
    RevColorSensorV3 colorSensor;
    RevTouchSensor touchSensor;
    DigitalChannel breakBeam;
    Servo led;
    Servo pusher;
    Outtake outtake;
    ITDCons.Color allianceColor;
    ITDCons.Color color;


    public static double INTAKE_POWER = -1;
    public static double EJECT_POWER = 1;

    public enum Status{
        TRANSFER(0), PICKUP_YELLOW(0), PICKUP_ALLIANCE(0), INIT(0), EJECT(1500),MOVE_TO_TRANSFER(500), EXTEND_TO_HUMAN(0), EJECT_TO_HUMAN(800), NEUTRAL(0);
        private final long time;
        Status(long time){
            this.time= time;
        }
        public long getTime() {
            return time;
        }
    }

    ElapsedTime elapsedTime = null;

    public  Status status;

    private int target;


    public Intake(Init init, Telemetry telemetry){
        this.telemetry = telemetry;
        this.init = init;

        intakeLeft = init.getIntakeLeft();
        intakeRight= init.getIntakeRight();
        intakeMotor = init.getIntake();
        extendo = init.getIntakeExtendo();
        colorSensor = init.getColor();
        breakBeam = init.getBreakBeam();
        led = init.getLed();
        pusher = init.getPusherServo();

        status = Status.INIT;
        color= ITDCons.Color.unknown;
        pidController = new PIDController(p,i,d);
        target =0;
    }

    public void setAllianceColor(ITDCons.Color color){
        allianceColor = color;
    }

    public void initStatusTeleop(){
        status = Status.NEUTRAL;
        color= ITDCons.Color.unknown;
        intakeToNeutral();
    }

    protected void pickupSample(){
        color= ITDCons.Color.unknown;
        dropIntake();
        startIntake();

    }
    public  void pickupSampleYellow(){
        pickupSample();
        status = Status.PICKUP_YELLOW;
    }

    public void pickupSampleAlliance(){
        pickupSample();
        status = Status.PICKUP_ALLIANCE;
    }

    public void toNeutral(){
        intakeToNeutral();
        stopIntake();
        if (status== Status.PICKUP_YELLOW){
            target =0;
        } else if (status == Status.PICKUP_ALLIANCE ){
            if (target == ITDCons.MaxExtension){
                target = ITDCons.halfExtension;
            } else if (target == ITDCons.halfExtension){
                target =0;
            }
        }
        status = Status.NEUTRAL;
    }

    public void toTransfer(){
        moveIntakeToTransfer();
        color = ITDCons.Color.yellow;
    }


    protected void startIntake (){
        intakeMotor.setPower(INTAKE_POWER);
    }

    public void stopIntake(){
        intakeMotor.setPower(0);
    }

    public void ejectIntake(){
        intakeMotor.setPower(EJECT_POWER);
        color = ITDCons.Color.unknown;
        elapsedTime= new ElapsedTime();
        status=Status.EJECT;
    }


    public void retractSlide() {
        target =0;
    }

    public void extendSlideHumanPlayer(){
        target = ITDCons.MaxExtension;
        dropIntake();
        status= Status.EXTEND_TO_HUMAN;

    }

    public void extendSlideHalf() {
        target = ITDCons.halfExtension;
        intakeToNeutral();
    }

    public void extendSlideMax(){
        target= ITDCons.MaxExtension;
        intakeToNeutral();
    }


    public void dropIntake(){
        intakeLeft.setPosition(ITDCons.intakeArmDrop);
        intakeRight.setPosition(ITDCons.intakeChainDrop);
    }

    protected void transferIntake(){
        intakeLeft.setPosition(ITDCons.intakeArmTransfer);
        intakeRight.setPosition(ITDCons.intakeChainTransfer);
        target=0;
    }

    protected void intakeToNeutral(){
        intakeLeft.setPosition(ITDCons.intakeArmNeutral);
        intakeRight.setPosition(ITDCons.intakeChainNeutral);
    }

    protected void moveIntakeToTransfer(){
        transferIntake();
        status = Status.MOVE_TO_TRANSFER;
        elapsedTime = new ElapsedTime();
    }

    public void pushOut(){ pusher.setPosition(ITDCons.pushOut); }

    public void pushIn(){ pusher.setPosition(ITDCons.pushIn); }

    public void update(){

            int pos = extendo.getCurrentPosition();
            double pid = pidController.calculate(pos, target);

            double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;

            double lift = pid + ff;

            extendo.setPower(lift);

            switch (status){
                case EXTEND_TO_HUMAN:
                    if (pos>ITDCons.MaxExtension-100) {
                        intakeMotor.setPower(ITDCons.intakeEjectSpeed);
                        status= Status.EJECT_TO_HUMAN;
                        elapsedTime = new ElapsedTime();
                        color= ITDCons.Color.unknown;
                    }
                    break;
                case EJECT_TO_HUMAN:
                    if (elapsedTime!=null && elapsedTime.milliseconds()> status.getTime()){
                        intakeMotor.setPower(0);
                        intakeToNeutral();
                        extendSlideHalf();
                        elapsedTime=null;
                        color= ITDCons.Color.unknown;
                    }
                    break;

                case EJECT:
                    if (elapsedTime!=null && elapsedTime.milliseconds()> status.getTime()){
                        intakeMotor.setPower(0);
                        intakeToNeutral();
                        elapsedTime=null;
                        if (color== ITDCons.Color.yellow){
                            target=0;
                        } else {
                            color= ITDCons.Color.unknown;
                        }
                    }
                    break;

                case PICKUP_YELLOW: //get samples from submersible

                    if (!breakBeam.getState()){
                        //read color
                        checkColor();
                        intakeMotor.setPower(0);
                        if (color!= ITDCons.Color.unknown && color!= ITDCons.Color.yellow && color!=allianceColor){
                            ejectIntake();
                        } else if (color== ITDCons.Color.yellow){
                            toTransfer();

                            status = Status.NEUTRAL;
                        } else if (color == allianceColor){
                           toTransfer();
                        }

                    } else{
                        led.setPosition(ITDCons.off);
                        color = ITDCons.Color.unknown;
                    }
                    break;
                case PICKUP_ALLIANCE:
                    if (!breakBeam.getState()){
                        //read color
                        checkColor();
                        intakeMotor.setPower(0);
                        if (color!= ITDCons.Color.unknown && (color== ITDCons.Color.yellow || color!=allianceColor)){
                            ejectIntake();
                        }  else if (color == allianceColor){
                            toNeutral();
                        }

                    } else{
                        led.setPosition(ITDCons.off);
                        color = ITDCons.Color.unknown;
                    }
                    break;
                case MOVE_TO_TRANSFER:
                    if (elapsedTime!=null && elapsedTime.milliseconds()> status.getTime()){
//                        intakeMotor.setPower(EJECT_POWER);
                       // status = Status.EJECT;
                      //  elapsedTime = new ElapsedTime();
                    }
                    break;
            }

    }

    public void setTarget(int target){
        this.target= target;
    }

    public void led(){
        led.setPosition(1);
    }


    public void checkColor(){

        if (colorSensor.red()>colorSensor.blue() && colorSensor.red()> colorSensor.green()){
            led.setPosition(ITDCons.red);
            color = ITDCons.Color.red;
        }
        else if (colorSensor.green()>colorSensor.blue() && colorSensor.green()>colorSensor.red()){
            led.setPosition(ITDCons.yellow);
            color = ITDCons.Color.yellow;
        } else if (colorSensor.blue()>colorSensor.green() && colorSensor.blue()>colorSensor.red()){
            led.setPosition(ITDCons.blue);
            color = ITDCons.Color.blue;
        } else {
            led.setPosition(ITDCons.off);
            color = ITDCons.Color.unknown;
        }

    }

    public ITDCons.Color getColor(){
        return color;
    }

    public Status getStatus(){
        return status;
    }

    public void setOuttake(Outtake outtake){
        this.outtake= outtake;
    }

    public boolean readyToTransfer(){
        return (color == ITDCons.Color.yellow && extendo.getCurrentPosition()<200 && status==Status.EJECT);
    }

    public void intakeintake(){
        intakeLeft.setPosition(ITDCons.intakeintakearm);
        intakeRight.setPosition(ITDCons.intakeintakechain);
    }


}
