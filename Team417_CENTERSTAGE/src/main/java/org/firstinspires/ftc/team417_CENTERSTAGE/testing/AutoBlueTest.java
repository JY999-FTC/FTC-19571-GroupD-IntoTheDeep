package org.firstinspires.ftc.team417_CENTERSTAGE.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team417_CENTERSTAGE.baseprograms.BaseAutonomous;

// Test class to run Auto without actually making the robot move; Used for OpenCv and April Tags

@Autonomous(name = "Auto Blue Test")

public class AutoBlueTest extends BaseAutonomous {
    public void runOpMode() {
        runAuto(false, true, true);
        while (opModeIsActive());
    }
}
