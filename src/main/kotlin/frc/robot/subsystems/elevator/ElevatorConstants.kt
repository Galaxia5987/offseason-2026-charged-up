package frc.robot.subsystems.elevator

import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import frc.robot.field.getPostByLevel
import frc.robot.lib.Gains
import frc.robot.lib.MotionMagicGains
import frc.robot.lib.createCurrentLimits
import frc.robot.lib.extensions.*
import org.team5987.annotation.command_enum.CommandEnum
import org.wpilib.units.measure.Distance
import kotlin.math.sin

const val MAIN_PORT = 1
const val AUX_PORT = 2
const val GEAR_RATIO = 1.0
val DIAMETER = 20.mm

val ELEVATOR_ANGLE = 27.773198.deg
val TOLERANCE = 1.cm
val HEIGHT_TOLERANCE = 0.5.m
val MAX_LENGTH = 1.m
val MIN_LENGTH = 0.m
val BASE_HEIGHT = 0.394.m

val MOTION_MAGIC_CONFIG = MotionMagicGains(80.rps, 160.rps_squared, 1600.0)
val GAINS = Gains(1.0, motionMagicGains = MOTION_MAGIC_CONFIG)
val SIM_GAINS = Gains(kP = 0.35, kD = 0.04)

val MOTOR_CONFIG =
    TalonFXConfiguration().apply {
        CurrentLimits = createCurrentLimits()
        MotorOutput =
            MotorOutputConfigs().apply {
                NeutralMode = NeutralModeValue.Brake
                Inverted = InvertedValue.Clockwise_Positive
            }
        Slot0 = GAINS.toSlotConfig()
        SoftwareLimitSwitch.apply {
            ForwardSoftLimitEnable = true
            ReverseSoftLimitEnable = true
            ForwardSoftLimitThreshold =
                MAX_LENGTH.toAngle(DIAMETER, GEAR_RATIO)[rot]
            ReverseSoftLimitThreshold =
                MIN_LENGTH.toAngle(DIAMETER, GEAR_RATIO)[rot]
        }
    }

fun calculateLength(level: Int): Distance = getPostByLevel(level).z.m / sin(ELEVATOR_ANGLE[rad])

@CommandEnum
enum class ElevatorHeights(val defaultLength: Distance) {
    CLOSE(calculateLength(0)),
    MID(calculateLength(1)),
    HIGH(calculateLength(2));

    val level
        get() = ordinal
}
