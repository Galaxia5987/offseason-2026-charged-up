package frc.robot.lib.sysid

import com.ctre.phoenix6.SignalLogger
import frc.robot.lib.commands.invoke
import frc.robot.lib.commands.unaryPlus
import frc.robot.lib.extensions.div
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.sec
import frc.robot.lib.extensions.volts
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber
import org.wpilib.command3.Command
import org.wpilib.command3.Mechanism
import org.wpilib.sysid.SysIdRoutineLog
import org.wpilib.units.VoltageUnit
import org.wpilib.units.measure.Time
import org.wpilib.units.measure.Velocity
import org.wpilib.units.measure.Voltage

private val TIME_BETWEEN_ROUTINES = 1.sec

data class SysIdRoutineConfig(
    val rampRate: Velocity<VoltageUnit>,
    val stepVoltage: Voltage,
    val timeout: Time,
    val direction: SysIdRoutine.Direction,
)

data class SysIdMechanismConfig(
    val forward: SysIdRoutineConfig,
    val backward: SysIdRoutineConfig,
)

/**
 * Extension function that creates a [SysIdCommand] for any subsystem that
 * implements [SysIdable] and [Mechanism].
 *
 * @return A [Command].
 */
fun <T> T.sysId(): Command where T : SysIdable, T : Mechanism =
    SysIdCommand(
            this,
            sysidConfig.forward,
            sysidConfig.backward,
        )
        .command()

/**
 * Interface that allows a subsystem to be characterized via SysId. Must provide
 * a method to set voltage on the subsystem.
 */
interface SysIdable {

    /** Applies the specified [voltage] to the subsystem. */
    fun setVoltage(voltage: Voltage)

    val sysidConfig: SysIdMechanismConfig
}

/**
 * Helper that generates a WPILib [Command] to run SysId routines
 * (forward/backward/quasistatic).
 *
 * @param T The subsystem type, which must implement [SysIdable] and extend
 *   [Mechanism].
 * @property subsystem The target subsystem being characterized.
 * @property forwardRoutineConfig configuration for the forward routine.
 * @property backwardRoutineConfig configuration for the backward routine.
 */
class SysIdCommand<T>(
    private val subsystem: T,
    forwardRoutineConfig: SysIdRoutineConfig,
    backwardRoutineConfig: SysIdRoutineConfig,
) where T : SysIdable, T : Mechanism {

    private val name = subsystem.name

    private val forwardRoutineConfig =
        LoggedSysIdRoutineConfig(forwardRoutineConfig)
    private val backwardRoutineConfig =
        LoggedSysIdRoutineConfig(backwardRoutineConfig)

    /**
     * Creates the [SysIdRoutine] object with the provided configuration.
     *
     * @param routineConfig A configuration for the routine.
     */
    private fun createRoutine(routineConfig: LoggedSysIdRoutineConfig) =
        SysIdRoutine(
            SysIdRoutine.Config(
                routineConfig.loggedRampRate.get().volts / sec,
                routineConfig.loggedStepVoltage.get().volts,
                routineConfig.loggedTimeout.get().sec,
            ) { state: SysIdRoutineLog.State ->
                SignalLogger.writeString("state", state.toString())
                Logger.recordOutput("SysId/$name/state", state.toString())
            },
            SysIdRoutine.SysIdMechanism(
                subsystem::setVoltage,
                null,
                subsystem,
            ),
        )

    /**
     * Builds the full characterization command sequence:
     * 1. Initializes routines
     * 2. Runs forward dynamic
     * 3. Runs backward dynamic
     * 4. Runs forward quasistatic
     * 5. Runs backward quasistatic
     *
     * Waits [TIME_BETWEEN_ROUTINES] between each step.
     *
     * @return The full [Command] sequence.
     */
    fun command(): Command =
        subsystem {
                val forwardRoutine = createRoutine(forwardRoutineConfig)
                val backwardRoutine = createRoutine(backwardRoutineConfig)
                +forwardRoutine.dynamic(SysIdRoutine.Direction.FORWARD)
                wait(TIME_BETWEEN_ROUTINES)
                +backwardRoutine.dynamic(SysIdRoutine.Direction.REVERSE)
                wait(TIME_BETWEEN_ROUTINES)
                +forwardRoutine.quasistatic(SysIdRoutine.Direction.FORWARD)
                wait(TIME_BETWEEN_ROUTINES)
                +backwardRoutine.quasistatic(SysIdRoutine.Direction.REVERSE)
            }
            .named("$name/characterize")

    /**
     * Holds the constants used for configuring a [SysIdRoutine], with tunable
     * logging support.
     */
    inner class LoggedSysIdRoutineConfig(val config: SysIdRoutineConfig) {

        val loggingPath = "/Tuning/SysId/$name/${config.direction.name}"

        /** Logged ramp rate value in volts/sec, tunable via NetworkTables. */
        val loggedRampRate =
            LoggedNetworkNumber(
                "$loggingPath/rampRate",
                config.rampRate.`in`(volts.per(sec)),
            )

        /** Logged step voltage in volts, tunable via NetworkTables. */
        val loggedStepVoltage =
            LoggedNetworkNumber(
                "$loggingPath/stepVoltage",
                config.stepVoltage[volts],
            )

        /** Logged timeout duration in seconds, tunable via NetworkTables. */
        val loggedTimeout =
            LoggedNetworkNumber("$loggingPath/timeout", config.timeout[sec])
    }
}
