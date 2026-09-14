package frc.robot.subsystems.wrist

import com.ctre.phoenix6.CANBus.systemcore
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.CANcoder
import frc.robot.lib.commands.UnnamedCommand
import frc.robot.lib.commands.addPeriodic
import frc.robot.lib.commands.invoke
import frc.robot.lib.commands.waitUntil
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.with
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger
import org.wpilib.command3.Mechanism
import org.wpilib.command3.Trigger

object Wrist : Mechanism(), WristPositionCommandFactory {

    private val externalEncoder =
        CANcoder(ENCODER_PORT, systemcore(0)).apply {
            configurator.apply(ENCODER_CONFIG)
        }

    private val motor: UniversalTalonFX =
        UniversalTalonFX(
            port = PORT,
            config = CONFIG,
            gearRatio = ROTOR_TO_MECHANISM_RATIO,
            simGains = SIM_GAINS,
        )

    var setpoint = 0.deg
    var state = WristPosition.CLOSED
    val positionVoltage = PositionVoltage(setpoint)
    val atSetpoint = Trigger {
        motor.inputs.position.isNear(setpoint, TOLERANCE)
    }

    override fun setTarget(value: WristPosition): UnnamedCommand = this {
        setpoint = value.angle
        state = value
        motor.setControl(positionVoltage with setpoint)
        atSetpoint.waitUntil()
    }

    init {
        addPeriodic(::periodic)
    }

    fun periodic() {
        motor.periodic()
        Logger.recordOutput("Subsystems/Wrist/setpoint", setpoint)
        Logger.recordOutput("Subsystems/Wrist/state", state)
        Logger.recordOutput("Subsystems/Wrist/atsetpoint", atSetpoint)
    }
}
