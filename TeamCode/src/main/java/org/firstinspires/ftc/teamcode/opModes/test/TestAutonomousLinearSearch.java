package org.firstinspires.ftc.teamcode.opModes.test;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.AutonomousModeBase;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.HardwareMapContainer;


@Autonomous(name="<OpMode name>", group="<OpMode group name>")
public class TestAutonomousLinearSearch extends AutonomousModeBase {
    // Declare class members here
    private ElapsedTime runtime = new ElapsedTime();
    RevIMU internal_measurement_unit;
    //left wheel
    Motor motor1 = HardwareMapContainer.motor0;
    //right wheel
    Motor motor2 = HardwareMapContainer.motor1;

    @Override
    public void run() {
        //Code for autonomous running
        //Coordinate in meters from start point in terms of x and y
        internal_measurement_unit = new RevIMU(HardwareMapContainer.getMap());
        internal_measurement_unit.init();
        //remember to account for size of robot when inputting coordinates.
        //Can we standardise placing these coordinates on the bottom left corner of the station
        double[][] junction_Coordinates = {};

        //Is it possible to use the terminals as a way to check coordinates?

        //remember to account for size of robot when inputting coordinates.
        //Can we standardise placing these coordinates at the center of the terminal
        double[][] terminal = {};
        //remember to account for size of robot when inputting coordinates.
        //Can we standardise placing these coordinates at the center of the terminal
        double[][] cone_coordinates={};
        double[] current = {0, 0};
        int i = 0;
        waitForStart();
        // Code to run once PLAY is pressed
        boolean has_cone = true;
        runtime.reset();
        //declaring idealheading
        double idealheading = 0.00;
        //Defining elapsed time
        double time = 0.0;
        // Run until the driver presses STOP
        while (opModeIsActive()) {
            // Code to run in a loop
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
            //getting time the previous tick started
            double start_elapsed = time;
            //converting runtime into a double
            time = runtime.seconds();
            //Telling the robot to automatically break from driving loop when 10s left to return to base
            while (time < 20) {
                double speed = motor1.getCorrectedVelocity();
                //finding time elapsed for previous tick
                double elapsed = time - start_elapsed;
                double travelled = speed * elapsed;
                //updating current x coordinate
                current[0] = current[0] + travelled / Math.tan(idealheading);
                //updating current y coordinate
                current[1] = current[1] + travelled * Math.tan(idealheading);

                //Distance from target in terms of x distance and y distance
                if (has_cone == false) {
                    //Run code to return to nearest loading station
                    //finding distance from each robot to cones using pythagoras
                    double[] closest_coordinate= {cone_coordinates[0][0], cone_coordinates[0][1]};
                    double smallest_distance = Math.pow(current[0] - cone_coordinates[0][0], 2);
                    smallest_distance = smallest_distance + Math.pow(current[1] - cone_coordinates[0][1],2);
                    //running through all the cone coordinates to find the coordinate with the
                    //lowest distance
                    for (int index = 1;i<cone_coordinates.length; i++){
                        double distance = Math.pow(current[0] - cone_coordinates[index][0], 2);
                        distance = distance + Math.pow(current[1] - cone_coordinates[index][1],2);
                        if (distance<smallest_distance){
                            smallest_distance=distance;
                            closest_coordinate=cone_coordinates[index];
                        }
                    }


                    idealheading= Math.atan2(current[1] - closest_coordinate[1], current[0] - closest_coordinate[0]);
                    //allowing an error of 1 degree as long as it is constantly updated.
                    double startheading = internal_measurement_unit.getAbsoluteHeading();
                    if (startheading < idealheading + 1 && startheading > idealheading - 1) {

                        //1000 is just a stopgap

                        motor1.setTargetPosition(1000);
                        motor2.setTargetPosition(1000);
                    } else if ((startheading - idealheading > 0) || (startheading - idealheading < -180)) {
                        //Cause the motor to turn left
                        motor1.set(-1);
                        motor2.set(1);

                        //10 degree error allowance for reducing speed is stopgap, need to input degree
                        //change generated by 1 tick of maximum turning

                        if (startheading - idealheading < 10 && startheading - idealheading > -10) {
                            //0.3 is a stopgap
                            motor1.set(-0.3);
                            motor2.set(0.3);
                        }
                    } else {
                        //Cause the motor to turn right
                        motor1.set(1);
                        motor2.set(-1);
                        if (startheading - idealheading < 10 && startheading - idealheading > -10) {
                            //0.3 is a stopgap
                            motor1.set(-0.3);
                            motor2.set(0.3);
                        }
                    }



                }
                //getting coordinates of next junction
                double[] next = junction_Coordinates[i];
                //creating coordinates of change in x distance and change in y distance necessary.
                double[] distance = {current[0] - next[0], current[1] - next[1]};
                //Calculating heading necessary to go in correct direction
                idealheading = Math.atan2(distance[1], distance[0]);
                //using pythagoras to calculate distanced needed to be travelled
                double travel_distance = distance[1] * distance[1] + distance[0] * distance[0];

                //find a way to calculate the ticks needed to travel this distance

                // Insert code for proximity sensor

                double startheading = internal_measurement_unit.getAbsoluteHeading();
                //allowing an error of 1 degree as long as it is constantly updated.
                if (startheading < idealheading + 1 && startheading > idealheading - 1) {

                    //1000 is just a stopgap

                    motor1.setTargetPosition(1000);
                    motor2.setTargetPosition(1000);
                } else if ((startheading - idealheading > 0) || (startheading - idealheading < -180)) {
                    //Cause the motor to turn left
                    motor1.set(-1);
                    motor2.set(1);

                    //10 degree error allowance for reducing speed is stopgap, need to input degree
                    //change generated by 1 tick of maximum turning

                    if (startheading - idealheading < 10 && startheading - idealheading > -10) {
                        //0.3 is a stopgap
                        motor1.set(-0.3);
                        motor2.set(0.3);
                    }
                } else {
                    //Cause the motor to turn right
                    motor1.set(1);
                    motor2.set(-1);
                    if (startheading - idealheading < 10 && startheading - idealheading > -10) {
                        //0.3 is a stopgap
                        motor1.set(-0.3);
                        motor2.set(0.3);
                    }
                }
            }
            while (time < 30) {
                //write code for returning to terminals
                //finding distance from each terminal to find most efficient path to terminal
                //using pythagoras
                double distance1 = Math.pow(current[0] - terminal[0][0], 2);
                distance1 = distance1 + Math.pow(current[1] - terminal[0][1],2);
                double distance2 = Math.pow(current[0] - terminal[1][0], 2);
                distance2 = distance2 +Math.pow(current[1] - terminal[1][1],2);
                if (distance1 > distance2) {
                    //if the second terminal is closer to the robot
                    double distance[] = {current[0] - terminal[0][0], current[1] - terminal[0][1]};
                    idealheading = Math.atan2(distance[1], distance[0]);
                } else {
                    //if the first terminal is closer to the robot
                    double distance[] = {current[0] - terminal[0][0], current[1] - terminal[0][1]};
                    idealheading = Math.atan2(distance[1], distance[0]);
                }
                double startheading = internal_measurement_unit.getAbsoluteHeading();
                //allowing an error of 1 degree as long as it is constantly updated.
                if (startheading < idealheading + 1 && startheading > idealheading - 1) {

                    //1000 is just a stopgap

                    motor1.setTargetPosition(1000);
                    motor2.setTargetPosition(1000);
                } else if ((startheading - idealheading > 0) || (startheading - idealheading < -180)) {
                    //Cause the motor to turn left
                    motor1.set(-1);
                    motor2.set(1);

                    //10 degree error allowance for reducing speed is stopgap, need to input degree
                    //change generated by 1 tick of maximum turning

                    if (startheading - idealheading < 10 && startheading - idealheading > -10) {
                        //0.3 is a stopgap
                        motor1.set(-0.3);
                        motor2.set(0.3);
                    }
                } else {
                    //Cause the motor to turn right
                    motor1.set(1);
                    motor2.set(-1);
                    if (startheading - idealheading < 10 && startheading - idealheading > -10) {
                        //0.3 is a stopgap
                        motor1.set(-0.3);
                        motor2.set(0.3);
                    }
                }

            }
        }
    }
}