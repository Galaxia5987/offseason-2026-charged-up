package frc.robot.subsystems.roller

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VoltageOut
import frc.robot.lib.commands.UnnamedCommand
import frc.robot.lib.commands.addPeriodic
import frc.robot.lib.commands.invoke
import frc.robot.lib.extensions.volts
import frc.robot.lib.extensions.with
import frc.robot.lib.universal_motor.MotorLogConfig
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger
import org.wpilib.command3.Command
import org.wpilib.command3.Mechanism
import org.wpilib.units.measure.Voltage

abstract class Roller(name: String, val config: RollerConfig) : Mechanism("${name}Roller") {
    protected val motor: UniversalTalonFX =
        UniversalTalonFX(
            port = config.motorPort,
            canbus = config.canBus,
            config = TalonFXConfiguration().apply {
                MotorOutput = config.motorOutput
                CurrentLimits = config.currentLimits

            },
            logConfig = MotorLogConfig(
                current = true,
                velocity = true,
                voltage = true
            )
        )

    protected var setpoint: Voltage = 0.volts
    protected val voltageOut = VoltageOut(setpoint)

    init {
        addPeriodic(::_periodic)
    }

    protected open fun forward(): UnnamedCommand = this {
        setpoint = config.forwardVoltage
        motor.setControl(voltageOut with setpoint)
    }

    protected open fun backward(): UnnamedCommand = this {
        setpoint = config.reverseVoltage
        motor.setControl(voltageOut with setpoint)
    }

    public open fun stop(): Command = this {
        setpoint = 0.volts
        motor.setControl(voltageOut with setpoint)
    }.named("${name}/stop")

    private fun _periodic() {
        motor.periodic()
        Logger.recordOutput("Subsystems/${name}/setpoint", setpoint)
        periodic()
    }

    protected open fun periodic() { }
}

