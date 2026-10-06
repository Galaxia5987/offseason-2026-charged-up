package frc.robot.states

import frc.robot.drive
import frc.robot.field.CubeColors
import frc.robot.field.SCORING_POSTS
import frc.robot.field.nearestPost
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
            closeWristIfNoElement()
        ]
        park()
    }
        .named("States/Idle")

fun closeWristIfNoElement() = command {
    if(!Sensors.intakeSensor.isPresent) {
        +Wrist.closed()
    }
}.named("States/closeWristIfNoElement")

fun intaking(): Command =
    command {
        +[
            Wrist.open(),
            IntakeRoller.intake(),
            ConveyorRoller.convey(),
            DispatchRoller.stop(),
            GripRoller.stop(),
        ]
        park()
    }
        .named("States/Intaking")

fun alignment(): Command  =
    runToPose(::nearestPost)
        .named("Drive/AlignToScoringPost")

private fun advance(): Command =
    command {
        if (Sensors.intakeSensor.isPresent) {
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

        waitUntil { !Sensors.dispatchSensor.isPresent }
    }
        .whenCanceled {
            command {
                +ConveyorRoller.stop()
                +DispatchRoller.dispatchHigh() // Reverse the roller
                waitUntil { Sensors.dispatchSensor.isPresent }
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

        when (Sensors.gripSensor.color) {
            CubeColors.RED -> +Elevator.high()
            CubeColors.YELLOW -> +Elevator.mid()
            else -> +GripRoller.release()
        }

        +Elevator.close()
    }
        .named("States/Scoring/High")
