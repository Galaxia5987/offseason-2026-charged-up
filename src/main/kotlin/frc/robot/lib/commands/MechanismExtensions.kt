package frc.robot.lib.commands

import frc.robot.lib.MechanismRegistry
import org.littletonrobotics.junction.Logger
import org.wpilib.command3.Mechanism
import org.wpilib.command3.Scheduler
import org.wpilib.system.RobotController
import org.wpilib.units.Units.Microseconds
import org.wpilib.units.Units.Millisecond

fun Mechanism.addPeriodic(function: Runnable) {
    val timeLogPath = "Subsystems/$name/loopTime"
    Scheduler.getDefault().addPeriodic {
        val startTime = RobotController.getTime()
        function.run()
        val totalTime =
            Millisecond.convertFrom(
                (RobotController.getTime() - startTime).toDouble(),
                Microseconds,
            )
        Logger.recordOutput(timeLogPath, totalTime)
    }
}

fun initializeAllMechanisms() {
    MechanismRegistry.allMechanisms.forEach {
        it.objectInstance
    }
}

// So build won't fail on a project without any mechanisms
object EmptyMechanism : Mechanism()
