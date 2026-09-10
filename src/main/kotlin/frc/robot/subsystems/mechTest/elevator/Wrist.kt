package frc.robot.subsystems.mechTest.elevator

import com.ctre.phoenix6.controls.PositionVoltage
import frc.robot.lib.Gains
import frc.robot.lib.commands.UnnamedCommand
import frc.robot.lib.commands.addPeriodic
import frc.robot.lib.commands.invoke
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.with
import frc.robot.lib.mechanism2d.MechanismBuilder
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger
import org.team5987.annotation.command_enum.CommandEnum
import org.wpilib.command3.Mechanism
import org.wpilib.units.measure.Angle
import org.wpilib.units.measure.Distance

@CommandEnum
enum class WristState(val angle: Angle) {
    UPRIGHT(90.deg),
    CLOSED(0.deg)
}

object Wrist : Mechanism(), WristStateCommandFactory {

    private val motor = UniversalTalonFX(
        0,
        simGains = Gains(0.3, 0.0, 0.1)
    )

    var setpoint: Angle = 0.0.deg
        private set

    private val positionVoltage = PositionVoltage(0.0)

    override fun setTarget(value: WristState): UnnamedCommand = this {
        setpoint = value.angle
        motor.setControl(positionVoltage with setpoint)
    }

    init {
        addPeriodic(::periodic)
    }

    object Visualizer : MechanismBuilder() {

        val wristSection = section("wristSection", { 0.5.m }, {motor.inputs.position+180.deg})

        override val mechanism = root(x=2.0,y=0.0) {
            wristSection.attach()
        }
    }

    fun periodic() {
        motor.periodic()
        Visualizer.update()
        Logger.recordOutput("Subsystems/Wrist/setpoint", setpoint)
    }
}