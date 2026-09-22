package frc.robot.states

import frc.robot.RobotContainer.buttons.intake
import frc.robot.RobotContainer.buttons.scoring
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

                allOf<State>() on intake switchTo INTAKING

                INTAKING on !intake switchTo IDLE

                IDLE on scoring.trigger switchTo ALIGNMENT
                // TODO: Add trigger in position for scoring with 'and' operator

                ALIGNMENT on scoring.trigger switchTo IDLE

                ALIGNMENT.onComplete switchTo COLOR_CHECK

                COLOR_CHECK on
                    Trigger { Sensors.DispatchSensor.isGreen } switchTo
                    SCORING_LOW

                COLOR_CHECK on
                    !Trigger { Sensors.DispatchSensor.isGreen } switchTo
                    SCORING_HIGH

                SCORING_LOW.onComplete switchTo IDLE

                [SCORING_LOW, SCORING_HIGH] on scoring.trigger switchTo IDLE

                SCORING_HIGH.onComplete switchTo IDLE
            }
    }
}
