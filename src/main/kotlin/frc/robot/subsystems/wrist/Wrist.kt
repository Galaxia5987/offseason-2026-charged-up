package frc.robot.subsystems.wrist

import com.ctre.phoenix6.controls.PositionVoltage
import frc.robot.lib.commands.UnnamedCommand
import frc.robot.lib.commands.addPeriodic
import frc.robot.lib.commands.invoke
import frc.robot.lib.commands.waitUntil
import frc.robot.lib.extensions.deg
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger
import org.wpilib.command3.Command
import org.wpilib.command3.Mechanism
import org.wpilib.command3.Trigger

object Wrist : Mechanism(), WristPositionCommandFactory {

    private val motor: UniversalTalonFX =
        UniversalTalonFX(
            port = PORT,
            config = CONFIG,
            gearRatio = RATIO,
            simGains = SIM_GAINS,
        )

    var setpoint = 0.deg
    val positionVoltage = PositionVoltage(setpoint)
    val atSetpoint = Trigger {
        motor.inputs.position.isNear(setpoint, TOLERANCE)
    }

    override fun setTarget(value: WristPosition): UnnamedCommand = this {
        setpoint = value.angle
        motor.setControl(positionVoltage.withPosition(setpoint))
        atSetpoint.waitUntil()
    }


    init {
        addPeriodic(::periodic)
    }

    fun periodic() {
        motor.periodic()
        Logger.recordOutput("Subsystem/Wrist/setpoint", setpoint)
        Logger.recordOutput("Subsystem/Wrist/atsetpoint", atSetpoint)
    }
}
