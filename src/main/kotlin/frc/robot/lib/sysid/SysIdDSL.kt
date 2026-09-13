package frc.robot.lib.sysid

import org.wpilib.units.VoltageUnit
import org.wpilib.units.measure.Time
import org.wpilib.units.measure.Velocity
import org.wpilib.units.measure.Voltage

@DslMarker
annotation class SysIdDsl

@SysIdDsl
class SysIdRoutineConfigBuilder {
    var rampRate: Velocity<VoltageUnit>? = null
    var stepVoltage: Voltage? = null
    var timeout: Time? = null

    var direction: SysIdRoutine.Direction? = null

    fun build(): SysIdRoutineConfig {
        return SysIdRoutineConfig(
            rampRate = requireNotNull(rampRate) { "rampRate is required" },
            stepVoltage = requireNotNull(stepVoltage) { "stepVoltage is required" },
            timeout = requireNotNull(timeout) { "timeout is required" },
            direction = requireNotNull(direction) { "direction is required" }
        )
    }
}

@SysIdDsl
class SysIdMechanismConfigBuilder {
    private var forwardConfig: SysIdRoutineConfig? = null
    private var backwardConfig: SysIdRoutineConfig? = null

    /** Configure the forward routine specifically. */
    fun forward(block: SysIdRoutineConfigBuilder.() -> Unit) {
        forwardConfig = SysIdRoutineConfigBuilder().apply {
            direction = SysIdRoutine.Direction.FORWARD
            block()
        }.build()
    }

    /** Configure the backward routine specifically. */
    fun backward(block: SysIdRoutineConfigBuilder.() -> Unit) {
        backwardConfig = SysIdRoutineConfigBuilder().apply {
            direction = SysIdRoutine.Direction.REVERSE
            block()
        }.build()
    }

    /**
     * Forward and backward routines share the same voltages and timeouts.
     */
    fun symmetric(block: SysIdRoutineConfigBuilder.() -> Unit) {
        forwardConfig = SysIdRoutineConfigBuilder()
            .apply(block)
            .apply { direction = SysIdRoutine.Direction.FORWARD }
            .build()

        backwardConfig = SysIdRoutineConfigBuilder()
            .apply(block)
            .apply { direction = SysIdRoutine.Direction.REVERSE }
            .build()
    }

    fun build(): SysIdMechanismConfig {
        return SysIdMechanismConfig(
            forwardRoutineConfig = requireNotNull(forwardConfig) { "Forward configuration is missing" },
            backwardRoutineConfig = requireNotNull(backwardConfig) { "Backward configuration is missing" }
        )
    }
}

fun buildSysIdConfig(block: SysIdMechanismConfigBuilder.() -> Unit): SysIdMechanismConfig {
    return SysIdMechanismConfigBuilder().apply(block).build()
}