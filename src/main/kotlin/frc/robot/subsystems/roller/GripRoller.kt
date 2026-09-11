package frc.robot.subsystems.roller

import com.ctre.phoenix6.CANBus.systemcore
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import frc.robot.lib.commands.onChange
import frc.robot.lib.extensions.volts
import org.wpilib.command3.Trigger

val GRIP_ROLLER_CONFIG =
    RollerConfig(
        forwardVoltage = 5.volts,
        reverseVoltage = (-5).volts,
        canBus = systemcore(0),
        motorPort = 0,
        motorOutput =
            MotorOutputConfigs().apply {
                Inverted = InvertedValue.CounterClockwise_Positive
                NeutralMode = NeutralModeValue.Brake
            },
    )

object GripRoller : Roller("Grip", GRIP_ROLLER_CONFIG) {
    private val stopTrigger: Trigger =
        Trigger { true }.onChange(stop()) // TODO: Replace with gripSensor

    fun grip() = super.forward().named("${name}/grip")

    fun release() = super.backward().named("${name}/release")
}
