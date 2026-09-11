package frc.robot.lib.mechanism2d

import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.m
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d
import org.wpilib.command3.Mechanism
import org.wpilib.units.measure.Angle
import org.wpilib.units.measure.Distance
import org.wpilib.util.Color8Bit

@DslMarker annotation class Mechanism2dDsl

@Mechanism2dDsl
class Section(
    val name: String,
    val length: () -> Double,
    val angle: () -> Double,
    lineWidth: Double = 6.0,
    color: Color8Bit = Color8Bit(235, 137, 52),
) {
    val ligament =
        LoggedMechanismLigament2d(name, length(), angle(), lineWidth, color)
    private val children = mutableListOf<Section>()

    fun update() {
        ligament.length = length()
        ligament.angle = angle()
        children.forEach { it.update() }
    }

    fun section(
        name: String,
        length: () -> Distance,
        angle: () -> Angle,
        lineWidth: Double = 6.0,
        color: Color8Bit = Color8Bit(235, 137, 52),
        block: Section.() -> Unit = {},
    ): Section {
        val child =
            Section(name, { length()[m] }, { angle()[deg] }, lineWidth, color)
        child.block()
        child.attach()
        return child
    }

    // Accepts nullable to intercept uninitialized properties during class
    // instantiation
    fun Section?.attach() {
        requireNotNull(this) {
            "Cannot attach a null Section. If you declared this section as a property, ensure it is defined BEFORE the parent section."
        }
        this@Section.children.add(this)
        this@Section.ligament.append(this.ligament)
    }
}

@Mechanism2dDsl
class Root(
    val name: String,
    val x: Double,
    val y: Double,
) {
    var mechanismRoot: LoggedMechanismRoot2d? = null
    private val children = mutableListOf<Section>()

    fun bind(mech2d: LoggedMechanism2d) {
        mechanismRoot = mech2d.getRoot(name, x, y)
        children.forEach { mechanismRoot!!.append(it.ligament) }
    }

    fun update() {
        children.forEach { it.update() }
    }

    fun section(
        name: String,
        length: () -> Distance,
        angle: () -> Angle,
        lineWidth: Double = 6.0,
        color: Color8Bit = Color8Bit(235, 137, 52),
        block: Section.() -> Unit = {},
    ): Section {
        val child =
            Section(name, { length()[m] }, { angle()[deg] }, lineWidth, color)
        child.block()
        child.attach()
        return child
    }

    // Accepts nullable to intercept uninitialized properties during class
    // instantiation
    fun Section?.attach() {
        requireNotNull(this) {
            "Cannot attach a null Section. If you declared this section as a property in your MechanismBuilder subclass, ensure it is defined BEFORE 'override val mechanism = root { ... }'."
        }
        this@Root.children.add(this)
        this@Root.mechanismRoot?.append(this.ligament)
    }
}

/**
 * A DSL builder for creating and logging 2D mechanism visualizations using
 * [LoggedMechanism2d].
 *
 * This provides a syntax to define roots and sections of a mechanism. It
 * supports dynamic tracking via suppliers for length and angle, automatically
 * updates states, and allows sharing components across different subsystems.
 *
 * ### DSL Components
 * - `root`: Defines the base coordinate (x, y) of the mechanism. Every builder
 *   must override [mechanism] with a root.
 * - `section`: Defines a physical component. It can be declared as a property
 *   or nested directly inside a root or another section.
 * - `attach()`: Appends a declared [Section] to a [Root] or parent [Section].
 *   Order matters: the section property must be defined *before* it is
 *   attached.
 * - `update()`: Propagates the latest states from the provided lambdas to the
 *   underlying [LoggedMechanism2d] and logs the output.
 *
 * ### Example Usage
 *
 * The following example demonstrates a multi-part mechanism where a `Wrist` is
 * attached to the end of an `Elevator`. Each subsystem manages its own logic
 * while linking their visualizations.
 *
 * ```kotlin
 * object Wrist : Mechanism(), WristStateCommandFactory {
 *     // ...
 *
 *     object Visualizer : MechanismBuilder() {
 *         // 1. Define the section as a property so other subsystems can access it.
 *         // Pass lambda providers for length and angle to allow automatic updates.
 *         val wristSection = section("wristSection", { 0.5.m }, { motor.inputs.position + 180.deg })
 *
 *         // 2. Define a localized root for viewing the wrist independently if needed.
 *         override val mechanism = root(x = 2.0, y = 0.0) {
 *             wristSection.attach()
 *         }
 *     }
 *
 *     init {
 *         addPeriodic(::periodic)
 *     }
 *
 *     fun periodic() {
 *         motor.periodic()
 *         Visualizer.update() // Logs the mechanism state for this subsystem
 *     }
 * }
 *
 * object Elevator : Mechanism(), ElevatorStateCommandFactory {
 *     // ...
 *
 *     object Visualizer : MechanismBuilder() {
 *         // 1. Define the elevator section and attach the wrist section to its end.
 *         val elevatorSection = section("elevatorSection", motor.inputs::distance, { 90.deg }) {
 *             Wrist.Visualizer.wristSection.attach()
 *         }
 *
 *         // 2. Define the main root and attach the base section.
 *         override val mechanism = root {
 *             elevatorSection.attach()
 *         }
 *     }
 *
 *     init {
 *         addPeriodic(::periodic)
 *     }
 *
 *     fun periodic() {
 *         motor.periodic()
 *         Visualizer.update() // Logs the combined elevator and wrist mechanism
 *     }
 * }
 * ```
 */
abstract class MechanismBuilder(
    width: Double = 3.0,
    height: Double = 3.0,
    backgroundColor: Color8Bit = Color8Bit(0, 0, 32),
    var logPath: String? = null,
) {
    abstract val mechanism: Root

    private val mech2d = LoggedMechanism2d(width, height, backgroundColor)
    private var isBound = false

    fun update(logPath: String) {
        if (!isBound) {
            mechanism.bind(mech2d)
            isBound = true
        }
        mechanism.update()
        Logger.recordOutput(logPath, mech2d)
    }

    context(m: Mechanism)
    fun update() {
        if (logPath == null) {
            logPath = "Subsystems/${m.name}/Mechanism2d"
        }
        logPath?.let { update(it) }
    }

    fun section(
        name: String,
        length: () -> Distance,
        angle: () -> Angle,
        lineWidth: Double = 6.0,
        color: Color8Bit = Color8Bit(235, 137, 52),
        block: Section.() -> Unit = {},
    ): Section =
        Section(name, { length()[m] }, { angle()[deg] }, lineWidth, color)
            .apply(block)

    fun section(
        name: String,
        length: Distance,
        angle: Angle,
        lineWidth: Double = 6.0,
        color: Color8Bit = Color8Bit(235, 137, 52),
        block: Section.() -> Unit = {},
    ): Section = section(name, { length }, { angle }, lineWidth, color, block)

    fun root(
        name: String = "root",
        x: Double = 0.0,
        y: Double = 0.0,
        block: Root.() -> Unit = {},
    ): Root = Root(name, x, y).apply(block)
}
