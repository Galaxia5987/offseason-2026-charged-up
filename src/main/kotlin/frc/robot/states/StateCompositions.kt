package frc.robot.states

import frc.robot.drive
import frc.robot.lib.commands.command
import frc.robot.lib.commands.unaryPlus
import frc.robot.subsystems.elevator.Elevator
import frc.robot.subsystems.roller.ConveyorRoller
import frc.robot.subsystems.roller.DispatchRoller
import frc.robot.subsystems.roller.GripRoller
import frc.robot.subsystems.roller.IntakeRoller
import frc.robot.subsystems.wrist.Wrist
import frc.robot.subsystems.wrist.WristPosition
import org.wpilib.command3.Command

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

fun alignment() : Command = command {
    // TODO
}.named("States/Alignment")

fun scoringLow(): Command = command {
    drive.lock()
    if (true) { // todo if cube in intake
        Wrist.setTarget(WristPosition.OPEN)
        +IntakeRoller.intake()
    }
    else {
        Wrist.setTarget(WristPosition.CLOSED)
    }

    +ConveyorRoller.convey()
    +DispatchRoller.dispatchLow()

    waitUntil { true } // cube leaves body

    // todo unlock drive
}.named("States/Scoring/Low")

