package frc.robot.subsystems.elevator

import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import frc.robot.lib.Gains
import frc.robot.lib.MotionMagicGains
import frc.robot.lib.createCurrentLimits
import frc.robot.lib.extensions.*
import org.team5987.annotation.command_enum.CommandEnum
import org.wpilib.units.measure.Distance
import kotlin.math.sin
import kotlin.math.tan

const val MAIN_PORT = 1
const val AUX_PORT = 2
const val GEAR_RATIO = 1.0
val DIAMETER = 20.mm

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
    }

val TOLERANCE = 0.1.m
val ELEVATOR_ANGLE = 20.deg

@CommandEnum
enum class ElevatorHeights(val height: Distance) {
    CLOSE(0.m),
    MID(0.4.m),
    HIGH(0.6.m);

    fun toElevatorLength(): Distance = this.height / sin(ELEVATOR_ANGLE[rad])
    fun toLength(): Distance = this.height / tan(ELEVATOR_ANGLE[rad])
}
