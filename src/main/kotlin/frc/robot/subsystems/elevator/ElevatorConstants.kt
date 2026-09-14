package frc.robot.subsystems.elevator

import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import frc.robot.field.CUBE_SIZE
import frc.robot.field.TowerXOffset
import frc.robot.lib.Gains
import frc.robot.lib.MotionMagicGains
import frc.robot.lib.createCurrentLimits
import frc.robot.lib.extensions.*
import org.team5987.annotation.command_enum.CommandEnum
import org.wpilib.units.measure.Distance

const val MAIN_PORT = 1
const val AUX_PORT = 2
const val GEAR_RATIO = 1.0
val DIAMETER = 20.mm

val ELEVATOR_ANGLE = 20.deg
val TOLERANCE = 1.cm
val MAX_LENGTH = 1.m
val MIN_LENGTH = 0.m

val MOTION_MAGIC_CONFIG = MotionMagicGains(80.rps, 160.rps_squared, 1600.0)
val GAINS = Gains(1.0, motionMagicGains = MOTION_MAGIC_CONFIG)
val SIM_GAINS = Gains(1.0)

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

@CommandEnum
enum class ElevatorHeights(val minHeight: Distance,val towerXOffset: TowerXOffset) {
    MID(CUBE_SIZE * 2.0, TowerXOffset.HIGH),
    HIGH(MID.minHeight + CUBE_SIZE, TowerXOffset.MID),
}
