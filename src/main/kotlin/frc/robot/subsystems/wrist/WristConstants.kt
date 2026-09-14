package frc.robot.subsystems.wrist

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MagnetSensorConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import frc.robot.lib.Gains
import frc.robot.lib.createCurrentLimits
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.rot
import org.team5987.annotation.command_enum.CommandEnum
import org.wpilib.units.measure.Angle

const val PORT = 1
const val ENCODER_PORT = 2
val SIM_GAINS = Gains(kP = 0.5, kD = 0.075)
val REAL_GAINS = Gains(kP = 0.5, kD = 0.075)
val TOLERANCE = 1.deg
const val ROTOR_TO_MECHANISM_RATIO = 1.0
const val SENSOR_TO_MECHANISM_RATIO = 1.0
val FORWARD_LIMIT = 90.deg
val REVERSE_LIMIT = 0.deg
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

        Feedback =
            FeedbackConfigs().apply {
                SensorToMechanismRatio = SENSOR_TO_MECHANISM_RATIO
                RotorToSensorRatio = ROTOR_TO_MECHANISM_RATIO
                FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder
                FeedbackRemoteSensorID = ENCODER_PORT
            }
    }

val ENCODER_CONFIG =
    CANcoderConfiguration().apply {
        MagnetSensor =
            MagnetSensorConfigs().apply {
                MagnetOffset = 0.25 // rot
            }
    }

@CommandEnum
enum class WristPosition(val angle: Angle) {
    CLOSED(0.deg),
    OPEN(90.deg),
}
