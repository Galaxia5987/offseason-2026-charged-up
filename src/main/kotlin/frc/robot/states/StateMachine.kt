package frc.robot.states

import frc.robot.RobotContainer
import frc.robot.lib.commands.unaryPlus
import frc.robot.lib.extensions.not
import frc.robot.lib.state_machine.StateMachineCompanion
import frc.robot.subsystems.sensors.Sensors
import org.wpilib.command3.Trigger

enum class State {
    IDLE,
    INTAKING,
    ALIGNMENT,
    SCORING_HIGH,
    SCORING_LOW,
    COLOR_CHECK;

    companion object : StateMachineCompanion<State>(State::class) {
        override val states = makeStates {
            IDLE(idle())
            INTAKING(intaking())
            SCORING_HIGH(scoringHigh())
            SCORING_LOW(scoringLow())
            COLOR_CHECK {
                park()
            }

            allOf<State>() on RobotContainer.Buttons.intake switchTo INTAKING

            INTAKING on !RobotContainer.Buttons.intake switchTo IDLE

            IDLE on RobotContainer.Buttons.scoring.trigger switchTo ALIGNMENT
            // TODO: Add trigger in position for scoring with 'and' operator

            ALIGNMENT on RobotContainer.Buttons.scoring.trigger switchTo IDLE

            ALIGNMENT.onComplete switchTo COLOR_CHECK

            COLOR_CHECK on
                Trigger { Sensors.DispatchSensor.isGreen } switchTo
                SCORING_LOW

            COLOR_CHECK on
                !Trigger { Sensors.DispatchSensor.isGreen } switchTo
                SCORING_HIGH

            SCORING_LOW.onComplete switchTo IDLE

            [SCORING_LOW, SCORING_HIGH] on
                RobotContainer.Buttons.scoring.trigger switchTo
                IDLE

            SCORING_HIGH.onComplete switchTo IDLE
        }
    }
}
