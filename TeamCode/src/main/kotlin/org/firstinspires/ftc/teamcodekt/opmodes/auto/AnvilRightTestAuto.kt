package org.firstinspires.ftc.teamcodekt.opmodes.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import ftc.rogue.blacksmith.Anvil
import ftc.rogue.blacksmith.Scheduler
import ftc.rogue.blacksmith.util.kt.toIn
import ftc.rogue.blacksmith.util.kt.toRad

@Autonomous
class AnvilRightTestAuto : RogueBaseAuto() {
    override fun gooo() {
        val startPose = Pose2d(91.toIn(), (-159).toIn(), 90.toRad())
        val startTraj = mainTraj(startPose)

        Anvil.startAutoWith(startTraj).onSchedulerLaunch()

        Scheduler.launchOnStart(opmode = this) {
            bot.updateComponents(mTelemetry)
            bot.drive.update()
            mTelemetry.update()
        }
    }

    private fun mainTraj(startPose: Pose2d): Anvil =
        Anvil.formTrajectory(bot.drive, startPose)
            .splineToSplineHeading(75.25, -17.0, 135.0, 117.5)

            .doTimes(5) {
                inReverse {
                    // Go to intaking position
                    splineTo(153.5, -29.5, 0.0)
                }
                // Go to deposit position
                splineToSplineHeading(77.25, -17.0, 135.0, 135.0)
            }
}
