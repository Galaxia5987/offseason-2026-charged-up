package frc.robot.subsystems.roller

import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import frc.robot.lib.createCurrentLimits
import org.wpilib.units.measure.Voltage

data class RollerConfig(
    val forwardVoltage: Voltage,
    val reverseVoltage: Voltage,
    val motorPort: Int,
    val canBus: CANBus,
    val motorOutput: MotorOutputConfigs,
    val currentLimits: CurrentLimitsConfigs = createCurrentLimits(),
)
