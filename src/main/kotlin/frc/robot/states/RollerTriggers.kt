package frc.robot.states

import frc.robot.subsystems.roller.ConveyorRoller
import frc.robot.subsystems.roller.DispatchRoller
import frc.robot.subsystems.roller.GripRoller
import frc.robot.subsystems.roller.IntakeRoller
import frc.robot.subsystems.sensors.Sensors.DispatchSensor
import frc.robot.subsystems.sensors.Sensors.GripSensor
import frc.robot.subsystems.sensors.Sensors.bodySensor
import org.wpilib.command3.Trigger

val intakeRollerStop =
    Trigger {
            bodySensor.isPresent && DispatchSensor.isPresent
        }
        .whileTrue(IntakeRoller.stop())

val conveyorRollerStop =
    Trigger {
            StateMachine.IDLE
        }
        .whileTrue(ConveyorRoller.stop())

val dispatchRollerStop =
    Trigger {
            (StateMachine.SCORING_HIGH && GripSensor.isPresent) ||
                (StateMachine.SCORING_LOW && StateMachine.cubeLeavesBody)
        }
        .whileTrue(DispatchRoller.stop())

val gripRollerStop =
    Trigger {
            (GripSensor.isPresent)
        }
        .whileTrue(GripRoller.stop())
