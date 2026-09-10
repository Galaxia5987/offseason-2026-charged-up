package frc.robot.subsystems.mechTest.elevator

import com.ctre.phoenix6.controls.PositionVoltage
import frc.robot.lib.Gains
import frc.robot.lib.commands.UnnamedCommand
import frc.robot.lib.commands.addPeriodic
import frc.robot.lib.commands.invoke
import frc.robot.lib.extensions.cm
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.toAngle
import frc.robot.lib.extensions.with
import frc.robot.lib.mechanism2d.MechanismBuilder
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput
import org.team5987.annotation.command_enum.CommandEnum
import org.wpilib.command3.Mechanism
import org.wpilib.units.measure.Distance

@CommandEnum
enum class ElevatorState(val distance: Distance) {
    HIGH(3.m),
    MID(2.m),
    LOW(1.m),
    ZERO(0.m)
}

object Elevator : Mechanism(), ElevatorStateCommandFactory {

    private val motor = UniversalTalonFX(
        0,
        simGains = Gains(0.2),
        linearSystemWheelDiameter = 3.cm
    )

    var setpoint: Distance = 0.0.m
        private set

    private val positionVoltage = PositionVoltage(0.0)

    object Visualizer : MechanismBuilder() {
        val elevatorSection = section("elevatorSection", motor.inputs::distance, {90.deg}) {
            Wrist.Visualizer.wristSection.attach()
        }

        override val mechanism = root {
            elevatorSection.attach()
        }
    }

    @LoggedOutput(LogLevel.COMP)
    val mechanism3d
        get() = Visualizer.mechanism.mechanismRoot?.generate3dMechanism()?.toTypedArray() ?: arrayOf()

    init {
        addPeriodic(::periodic)
    }

    fun periodic() {
        motor.periodic()
        Visualizer.update()
        Logger.recordOutput("Subsystems/Elevator/setpoint", setpoint)
    }

    override fun setTarget(value: ElevatorState): UnnamedCommand = this {
        setpoint = value.distance
        motor.setControl(positionVoltage with setpoint.toAngle(3.cm, 1.0))
    }
}