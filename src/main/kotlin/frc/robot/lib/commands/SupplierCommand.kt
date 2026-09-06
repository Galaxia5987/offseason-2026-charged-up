package frc.robot.lib.commands

import org.wpilib.command3.Command
import org.wpilib.command3.Coroutine
import org.wpilib.command3.Mechanism

class SupplierCommandScope(
    val coroutine: Coroutine,
    val isContinuous: Boolean,
) {
    /**
     * Executes the given block only if the command was invoked with a static
     * target. Use this to block execution (e.g., waiting for a setpoint) since
     * supplier commands are meant to run continuously without blocking.
     */
    inline fun whenOneShot(block: Coroutine.() -> Unit) {
        if (!isContinuous) {
            coroutine.block()
        }
    }

    /**
     * Executes the given block only if the command was invoked with a
     * continuous supplier.
     */
    inline fun whenContinuous(block: Coroutine.() -> Unit) {
        if (isContinuous) {
            coroutine.block()
        }
    }
}

class SupplierCommand<T>(
    private val mechanism: Mechanism,
    private val action: SupplierCommandScope.(T) -> Unit,
) {
    private var commandName: String? = null
    private var configurationHooks: (UnnamedCommand.() -> Unit)? = null

    operator fun invoke(target: T): Command =
        mechanism {
                SupplierCommandScope(this, isContinuous = false).action(target)
            }
            .apply { configurationHooks?.invoke(this) }
            .also {
                require(commandName != null) {
                    "SupplierCommand's name cannot be null!"
                }
            }
            .named(commandName)

    operator fun invoke(supplier: () -> T): Command =
        mechanism {
                val scope = SupplierCommandScope(this, isContinuous = true)
                while (true) {
                    scope.action(supplier())
                    yield()
                }
            }
            .apply { configurationHooks?.invoke(this) }
            .also {
                require(commandName != null) {
                    "SupplierCommand's name cannot be null!"
                }
            }
            .named("${commandName}WithSupplier")

    fun named(name: String): SupplierCommand<T> {
        commandName = name
        return this
    }

    fun configure(
        configurationHooks: UnnamedCommand.() -> Unit
    ): SupplierCommand<T> {
        this.configurationHooks = configurationHooks
        return this
    }
}

/**
 * Creates a command generator that supports both one shot static targets and
 * continuous dynamic suppliers from a single logic block.
 *
 * By defining the hardware interaction once, this builder generates a
 * [SupplierCommand] with overloaded `invoke` operators. This allows the
 * resulting property to be called as either a standard one shot command or a
 * continuously looping command.
 *
 * Use [SupplierCommandScope.whenOneShot] to define blocking exit conditions
 * (such as waiting for a mechanism to reach a setpoint) that should only occur
 * during static execution. Supplier commands bypass these blocks to prevent
 * stalling the continuous loop.
 *
 * @param action The core logic to apply the target value. This executes exactly
 *   once for static targets, and continuously on every tick for suppliers.
 * @return A [SupplierCommand] generator that can be customized with `.named()`
 *   and `.configure()`.
 *
 * Example:
 * ```kotlin
 * val setAngle = supplierCommand<Angle> { angle ->
 *     setpoint = constraintTurretLimit(angle)
 *     motor.setControl(positionVoltage.withPosition(setpoint))
 *
 *     whenOneShot {
 *         atSetpoint.waitUntil()
 *     }
 * }.configure {
 *   withPriority(2)
 *   whenCancelled { println("Command Cancelled!") }
 * }.named("Subsystems/Turret/setAngle")
 *
 * // Usage:
 * // val staticCmd = turret.setAngle(90.degrees)
 * // val dynamicCmd = turret.setAngle { SetpointManager.turretSetpoint }
 * ```
 */
fun <T> Mechanism.supplierCommand(action: SupplierCommandScope.(T) -> Unit) =
    SupplierCommand(this, action)
