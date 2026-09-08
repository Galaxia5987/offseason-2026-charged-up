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

@DslMarker
annotation class Mechanism2dDsl

@Mechanism2dDsl
class Section(
    val name: String,
    val length: () -> Double,
    val angle: () -> Double,
    lineWidth: Double = 6.0,
    color: Color8Bit = Color8Bit(235, 137, 52)
) {
    val ligament = LoggedMechanismLigament2d(name, length(), angle(), lineWidth, color)
    private val children = mutableListOf<Section>()

    fun attach(child: Section) {
        children.add(child)
        ligament.append(child.ligament)
    }

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
        block: Section.() -> Unit = {}
    ): Section = Section(name, { length()[m] }, { angle()[deg] }, lineWidth, color).apply {
        block()
        this@Section.attach(this)
    }

    fun Section.attach() {
        attach(this)
    }
}

@Mechanism2dDsl
class Root(
    val name: String,
    val x: Double,
    val y: Double
) {
    private var mechanismRoot: LoggedMechanismRoot2d? = null
    private val children = mutableListOf<Section>()

    fun attach(child: Section) {
        children.add(child)
        mechanismRoot?.append(child.ligament)
    }

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
        block: Section.() -> Unit = {}
    ): Section = Section(name, { length()[m] }, { angle()[deg] }, lineWidth, color).apply {
        block()
        this@Root.attach(this)
    }

    fun Section.attach() {
        attach(this)
    }
}

abstract class MechanismBuilder(
    width: Double = 3.0,
    height: Double = 3.0,
    backgroundColor: Color8Bit = Color8Bit(0, 0, 32),
    var logPath: String? = null
) {
    abstract val mechanism: Root

    private val mech2d = LoggedMechanism2d(width, height, backgroundColor)

    init {
        mechanism.bind(mech2d)
    }

    fun update(logPath: String) {
        mechanism.update()
        Logger.recordOutput(logPath, mech2d)
    }

    context(m: Mechanism)
    fun update() {
        if(logPath == null) {
            logPath = "Subsystems/${m.name}/Mechanism2d"
        }
        logPath?.let { update(it) }
    }

    fun section(
        name: String, length: () -> Distance, angle: () -> Angle,
        lineWidth: Double = 6.0, color: Color8Bit = Color8Bit(235, 137, 52),
        block: Section.() -> Unit = {}
    ): Section = Section(name, { length()[m] }, { angle()[deg] }, lineWidth, color).apply(block)

    fun section(
        name: String, length: Distance, angle: Angle,
        lineWidth: Double = 6.0, color: Color8Bit = Color8Bit(235, 137, 52),
        block: Section.() -> Unit = {}
    ): Section = section(name, { length }, { angle }, lineWidth, color, block)

    fun root(
        name: String = "root", x: Double = 0.0, y: Double = 0.0,
        block: Root.() -> Unit = {}
    ): Root = Root(name, x, y).apply(block)
}