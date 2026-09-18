package frc.robot.states

import frc.robot.drive
import frc.robot.field.CubeColors
import frc.robot.field.SCORING_POSTS
import frc.robot.lib.align.runToPose
import frc.robot.lib.commands.*
import frc.robot.subsystems.elevator.Elevator
import frc.robot.subsystems.roller.ConveyorRoller
import frc.robot.subsystems.roller.DispatchRoller
import frc.robot.subsystems.roller.GripRoller
import frc.robot.subsystems.roller.IntakeRoller
import frc.robot.subsystems.sensors.Sensors
import frc.robot.subsystems.wrist.Wrist
import org.wpilib.command3.Command

fun idle(): Command =
    command {
            +[
                IntakeRoller.stop(),
                ConveyorRoller.stop(),
                DispatchRoller.stop(),
                GripRoller.stop(),
                Elevator.close(),
            ]
        }
        .named("States/Idle")

fun intaking(): Command =
    command {
            +[
                Wrist.open(),
                IntakeRoller.intake(),
                ConveyorRoller.convey(),
                DispatchRoller.stop(),
                GripRoller.stop(),
            ]
        }
        .named("States/Intaking")

fun alignment(): Command =
    runToPose({
            drive.pose.nearest(SCORING_POSTS.get())
        })
        .named("Drive/AlignToScoringPost")

private fun advance(): Command =
    command {
            if (Sensors.IntakeSensor.isPresent) {
                +Wrist.open()
                +IntakeRoller.intake()
            } else {
                +Wrist.closed()
            }

            +ConveyorRoller.convey()
        }
        .named("States/advance")

fun scoringLow(): Command =
    command {
            drive.continousLock().fork()

            +advance()

            +DispatchRoller.dispatchLow()

            waitUntil { !Sensors.DispatchSensor.isPresent }
        }
        .whenCanceled {
            command {
                    +ConveyorRoller.stop()
                    +DispatchRoller.dispatchHigh() // Reverse the roller
                    waitUntil { Sensors.DispatchSensor.isPresent }
                    +DispatchRoller.stop()
                    +idle()
                }
                .withPriority(Command.HIGHEST_PRIORITY)
                .named("States/Scoring/Low/WhenCancelled")
                .schedule()
        }
        .named("States/Scoring/Low")

fun scoringHigh(): Command =
    command {
            drive.continousLock().fork()

            +advance()

            +DispatchRoller.dispatchHigh()
            +GripRoller.grip()

            GripRoller.stopTrigger.waitUntil()

            when (Sensors.GripSensor.color) {
                CubeColors.RED -> +Elevator.high()
                CubeColors.YELLOW -> +Elevator.mid()
                else -> +GripRoller.release()
            }

            +Elevator.close()
        }
        .named("States/Scoring/High")
