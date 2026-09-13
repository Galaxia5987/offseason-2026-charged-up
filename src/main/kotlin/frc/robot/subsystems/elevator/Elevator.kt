package frc.robot.subsystems.elevator

import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC
import com.ctre.phoenix6.signals.MotorAlignmentValue
import frc.robot.drive
import frc.robot.field.TOWER_GRID
import frc.robot.lib.commands.UnnamedCommand
import frc.robot.lib.commands.addPeriodic
import frc.robot.lib.commands.invoke
import frc.robot.lib.extensions.*
import frc.robot.lib.universal_motor.UniversalTalonFX
import kotlin.math.absoluteValue
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.tan
import org.wpilib.command3.Command
import org.wpilib.command3.Mechanism
import org.wpilib.command3.Trigger
import org.wpilib.units.measure.Distance

object Elevator : Mechanism(), ElevatorHeightsCommandFactory {
    private val mainMotor =
        UniversalTalonFX(
            MAIN_PORT,
            config = MOTOR_CONFIG,
            gearRatio = GEAR_RATIO,
            simGains = SIM_GAINS,
            linearSystemWheelDiameter = DIAMETER,
        )
    private val auxMotor =
        UniversalTalonFX(
                AUX_PORT,
                config = MOTOR_CONFIG,
                gearRatio = GEAR_RATIO,
                simGains = SIM_GAINS,
                linearSystemWheelDiameter = DIAMETER,
            )
            .apply {
                setControl(
                    Follower(mainMotor.port, MotorAlignmentValue.Opposed)
                )
            }

    val height: Distance by periodic {
        mainMotor.inputs.distance * sin(ELEVATOR_ANGLE[rad])
    }

    val targetLength: Distance by periodic {
        (drive.pose.x - TOWER_GRID.x).absoluteValue.m / cos(ELEVATOR_ANGLE[rad])
    }

    val targetHeight: Distance by periodic {
        (drive.pose.x - TOWER_GRID.x).absoluteValue.m * tan(ELEVATOR_ANGLE[rad])
    }

    private var setpoint = 0.m
    private val torqueCurrentFOC = MotionMagicTorqueCurrentFOC(0.0)

    val atSetpoint = Trigger {
        setpoint.isNear(mainMotor.inputs.distance, TOLERANCE)
    }

    init {
        addPeriodic(::periodic)
    }

    private val logList =
        PropertyLogGroup(
            ::atSetpoint,
            ::setpoint,
            ::targetLength,
            ::height,
            ::targetHeight,
        )

    fun periodic() {
        mainMotor.periodic()
        auxMotor.periodic()
        logList.periodic()
    }

    fun close(): Command =
        this {
                setpoint = 0.m
                mainMotor.setControl(
                    torqueCurrentFOC with
                        MIN_LENGTH.toAngle(DIAMETER, GEAR_RATIO)
                )
            }
            .named("close")

    override fun setTarget(value: ElevatorHeights): UnnamedCommand = this {
        while (true) {
            if (targetHeight >= value.minHeight) {
                setpoint = targetLength
                mainMotor.setControl(
                    torqueCurrentFOC with
                        targetLength.toAngle(DIAMETER, GEAR_RATIO)
                )
            }
            yield()
        }
    }
}
