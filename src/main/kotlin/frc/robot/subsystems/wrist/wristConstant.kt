package frc.robot.subsystems.wrist

import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import frc.robot.lib.Gains
import frc.robot.lib.createCurrentLimits
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.rot
import org.team5987.annotation.command_enum.CommandEnum
import org.wpilib.units.measure.Angle

val PORT = 1
val SIM_GAINS = Gains(kP = 0.5, kD = 0.075)
val REAL_GAINS = Gains(kP = 0.5, kD = 0.075)
val TOLERANCE = 1.deg
const val RATIO = 0.0
val FORWARD_LIMIT = 90.deg
val REVERSE_LIMIT = 0.deg
val CANcCoder = CANcoder(1, CANBus())
val CONFIG =
    TalonFXConfiguration().apply {
        CurrentLimits = createCurrentLimits()
        Slot0 = REAL_GAINS.toSlotConfig()
        MotorOutput =
            MotorOutputConfigs().apply {
                Inverted = InvertedValue.CounterClockwise_Positive
                NeutralMode = NeutralModeValue.Brake
            }
        SoftwareLimitSwitch =
            SoftwareLimitSwitchConfigs().apply {
                ForwardSoftLimitEnable = true
                ForwardSoftLimitThreshold =
                    FORWARD_LIMIT[rot] // The thresholds work in rotations.
                ReverseSoftLimitEnable = true
                ReverseSoftLimitThreshold = REVERSE_LIMIT[rot]
            }
        CurrentLimits = createCurrentLimits()
    }

@CommandEnum
enum class WristPosition(val angle: Angle) {
    CLOSED(0.deg),
    OPEN(90.deg),
}
