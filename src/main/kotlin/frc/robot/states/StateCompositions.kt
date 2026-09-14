package frc.robot.states

import frc.robot.drive
import frc.robot.field.CubeColors
import frc.robot.field.SCORING_POSTS
import frc.robot.lib.align.runToPose
import frc.robot.lib.commands.command
import frc.robot.lib.commands.fork
import frc.robot.lib.commands.unaryPlus
import frc.robot.lib.commands.waitUntil
import frc.robot.lib.extensions.toPose
import frc.robot.lib.getClosest
import frc.robot.subsystems.elevator.Elevator
import frc.robot.subsystems.elevator.ElevatorHeights
import frc.robot.subsystems.roller.ConveyorRoller
import frc.robot.subsystems.roller.DispatchRoller
import frc.robot.subsystems.roller.GripRoller
import frc.robot.subsystems.roller.IntakeRoller
import frc.robot.subsystems.sensors.Sensors
import frc.robot.subsystems.wrist.Wrist
import frc.robot.subsystems.wrist.WristPosition
import org.wpilib.command3.Command
import java.awt.Color

fun idle(): Command = command {
    +IntakeRoller.stop()
    +ConveyorRoller.stop()
    +DispatchRoller.stop()
    +GripRoller.stop()
    +Elevator.close()
}.named("States/Idle")

fun intaking(): Command = command {
    Wrist.setTarget(WristPosition.OPEN)
    +IntakeRoller.intake()
    +ConveyorRoller.convey()
    +DispatchRoller.stop()
    +GripRoller.stop()
}.named("States/Intaking")

fun alignment(): Command =
    command {
        +runToPose(SCORING_POSTS.getClosest(drive.pose.translation).toPose()).named("Drive/AlignToScoringPost")
    }
        .named("States/Alignment")

fun scoringLow(): Command = command {
    drive.continousLock().fork()

    if (Sensors.IntakeSensor.isPresent) {
        Wrist.setTarget(WristPosition.OPEN)
        +IntakeRoller.intake()
    }
    else {
        Wrist.setTarget(WristPosition.CLOSED)
    }

    +ConveyorRoller.convey()
    +DispatchRoller.dispatchLow()

    waitUntil { !Sensors.DispatchSensor.isPresent }
}.named("States/Scoring/Low")

fun scoringHigh(): Command = command {
    drive.continousLock().fork()

    if (Sensors.IntakeSensor.isPresent) {
        Wrist.setTarget(WristPosition.OPEN)
        +IntakeRoller.intake()
    }
    else {
        Wrist.setTarget(WristPosition.CLOSED)
    }

    +ConveyorRoller.convey()
    +DispatchRoller.dispatchHigh()
    +GripRoller.grip()

    GripRoller.stopTrigger.waitUntil()

    if (Sensors.GripSensor.color == CubeColors.RED)
        Elevator.setTarget(ElevatorHeights.HIGH)
    else if (Sensors.GripSensor.color == CubeColors.YELLOW)
        Elevator.setTarget(ElevatorHeights.MID)
    else

    +GripRoller.release()

    GripRoller.stopTrigger.waitUntil()

    +Elevator.close()
}.named("States/Scoring/High")
