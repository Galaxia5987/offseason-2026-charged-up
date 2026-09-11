package frc.robot.subsystems.roller

import com.ctre.phoenix6.CANBus.systemcore
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import frc.robot.lib.extensions.volts

val DISPATCH_ROLLER_CONFIG = RollerConfig(
    forwardVoltage = 5.volts,
    reverseVoltage = 5.volts,
    canBus = systemcore(0),
    motorPort = 0,
    motorOutput = MotorOutputConfigs().apply {
        Inverted = InvertedValue.CounterClockwise_Positive
        NeutralMode = NeutralModeValue.Brake
    },
)

object DispatchRoller : Roller("Dispatch", DISPATCH_ROLLER_CONFIG) {
    fun dispatchLow() = super.forward().named("${name}/dispatchLow")
    fun dispatchHigh() = super.backward().named("${name}/dispatchHigh")
}