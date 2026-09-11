package frc.robot

import frc.robot.lib.Mode
import org.littletonrobotics.junction.LoggedRobot
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput
import org.wpilib.command3.Trigger
import org.wpilib.driverstation.RobotState

const val LOOP_TIME = 0.02 // [s]

val logLevel = LogLevel.DEBUG

@LoggedOutput(LogLevel.COMP)
val CURRENT_MODE: Mode
    get() =
        if (LoggedRobot.isReal()) {
            Mode.REAL
        } else {
            if (System.getenv("isReplay") == "true") {
                Mode.REPLAY
            } else {
                Mode.SIM
            }
        }

val isSim = Trigger { CURRENT_MODE == Mode.SIM }

val isAuto = Trigger { RobotState.isAutonomous() } // TODO!!!

val isEnabled = Trigger { RobotState.isEnabled() } // TODO!!
