package frc.robot.states

import frc.robot.RobotContainer.buttons.intakeButton
import frc.robot.RobotContainer.buttons.scoringButton
import frc.robot.field.CubeColors
import frc.robot.lib.commands.unaryPlus
import frc.robot.lib.extensions.not
import frc.robot.lib.state_machine.buildStateMachine
import frc.robot.subsystems.sensors.Sensors
import org.team5987.annotation.graph.graphgen.GenerateStateMachineGraph
import org.wpilib.command3.Trigger

enum class State {
    IDLE,
    INTAKING,
    ALIGNMENT,
    SCORING_HIGH,
    SCORING_LOW,
    COLOR_CHECK;

    companion object {
        private val isGreen = Trigger {
            Sensors.DispatchSensor.color == CubeColors.GREEN
        }

        @GenerateStateMachineGraph("State")
        val stateMachine =
            buildStateMachine<State>("State Machine") {
                IDLE(idle())
                INTAKING(intaking())
                SCORING_HIGH(scoringHigh())
                SCORING_LOW(scoringLow())
                COLOR_CHECK {
                    park()
                }

                allOf<State>() on intakeButton switchTo INTAKING

                INTAKING on !intakeButton switchTo IDLE

                IDLE on scoringButton.trigger switchTo ALIGNMENT
                // TODO: Add trigger in position for scoring with 'and' operator

                ALIGNMENT on scoringButton.trigger switchTo IDLE

                ALIGNMENT.onComplete switchTo COLOR_CHECK

                COLOR_CHECK on isGreen switchTo SCORING_LOW

                COLOR_CHECK on !isGreen switchTo SCORING_HIGH

                SCORING_LOW.onComplete switchTo IDLE

                SCORING_LOW on scoringButton.trigger switchTo IDLE

                SCORING_HIGH.onComplete switchTo IDLE

                SCORING_HIGH on scoringButton.trigger switchTo IDLE
            }
    }
}
