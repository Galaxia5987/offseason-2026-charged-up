package frc.robot.states

import frc.robot.subsystems.roller.ConveyorRoller
import frc.robot.subsystems.roller.DispatchRoller
import frc.robot.subsystems.roller.GripRoller
import frc.robot.subsystems.roller.IntakeRoller
import frc.robot.subsystems.sensors.Sensors.bodySensor
import frc.robot.subsystems.sensors.Sensors.dispatchSensor
import frc.robot.subsystems.sensors.Sensors.gripSensor
import org.wpilib.command3.Trigger

fun initRollerTriggers() = Unit

val notIntaking =
    Trigger {
            bodySensor.isPresent && dispatchSensor.isPresent
        }
        .whileTrue(IntakeRoller.stop())

val conveyorRollerStop =
    State.trigger(State.IDLE).whileTrue(ConveyorRoller.stop())

val dispatchRollerStop =
    Trigger {
            (State.state == State.SCORING_HIGH && gripSensor.isPresent) ||
                (State.state == State.SCORING_LOW && !dispatchSensor.isPresent)
        }
        .whileTrue(DispatchRoller.stop())

val gripRollerStop =
    Trigger {
            (gripSensor.isPresent)
        }
        .whileTrue(GripRoller.stop())
