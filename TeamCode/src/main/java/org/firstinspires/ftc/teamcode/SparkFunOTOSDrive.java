package org.firstinspires.ftc.teamcode;



import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/**
 * Experimental extension of MecanumDrive that uses the SparkFun OTOS sensor for localization.
 * <p>
 * Released under the BSD 3-Clause Clear License by j5155 from 12087 Capital City Dynamics
 * Portions of this code made and released under the MIT License by SparkFun
 * Unless otherwise noted, comments are from SparkFun
 */
public class SparkFunOTOSDrive extends MecanumDrive {
    public static class Params {
        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).

        // RR localizer note: these units are inches and radians
        public SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, Math.toRadians(0));

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        public double linearScalar = 1.0;
        public double angularScalar = 1.0;
    }

    public static SparkFunOTOSDrive.Params PARAMS = new SparkFunOTOSDrive.Params();
    SparkFunOTOS otos;

    public SparkFunOTOSDrive(HardwareMap hardwareMap, Pose2d pose) {
        super(hardwareMap, pose);
        otos = hardwareMap.get(SparkFunOTOS.class,"sensor_otos");

        otos.setLinearUnit(SparkFunOTOS.LinearUnit.INCHES);
        otos.setAngularUnit(SparkFunOTOS.AngularUnit.RADIANS);

        otos.setOffset(PARAMS.offset);
        otos.setLinearScalar(PARAMS.linearScalar);
        otos.setAngularScalar(PARAMS.angularScalar);
        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your programs. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total

        // RR localizer note: numSamples number completely arbitrary at the moment, feel free to change to fit your needs
        // Will get better number once I actually get this sensor
        otos.calibrateImu(100, true);
    }
    @Override
    public PoseVelocity2d updatePoseEstimate() {
        SparkFunOTOS.Pose2D otosPose = otos.getPosition();
        SparkFunOTOS.Pose2D otosVel = otos.getVelocity();
        pose = OTOSPoseToRRPose(otosPose);

        // rr standard
        poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        FlightRecorder.write("ESTIMATED_POSE", new PoseMessage(pose));

        // RR localizer note:
        // TODO: sussy code
        // unsure how to even do this properly or whether this is the right way to do it
        // I don't know enough math to understand dual nums :(

        return new PoseVelocity2d(new Vector2d(otosVel.x, otosVel.y),otosVel.h);
    }
    /**
     * Convert SparkFunOTOS poses to Roadrunner poses
     * NOTE: assumes units are inches and radians
     * @param otosPose The SparkFun OTOS sensor's pose
     * @return Input pose as a Roadrunner pose
     */
    public Pose2d OTOSPoseToRRPose(SparkFunOTOS.Pose2D otosPose) {
        return new Pose2d(otosPose.x, otosPose.y, otosPose.h);
    }
}
