package frc.robot.subsystems.elevator

import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC
import com.ctre.phoenix6.signals.MotorAlignmentValue
import frc.robot.drive
import frc.robot.field.TowerXOffset
import frc.robot.field.getGridOffset
import frc.robot.lib.commands.UnnamedCommand
import frc.robot.lib.commands.addPeriodic
import frc.robot.lib.commands.invoke
import frc.robot.lib.commands.waitUntil
import frc.robot.lib.extensions.*
import frc.robot.lib.sysid.SysIdMechanismConfig
import frc.robot.lib.sysid.SysIdRoutine
import frc.robot.lib.sysid.SysIdRoutineConfig
import frc.robot.lib.sysid.SysIdable
import frc.robot.lib.universal_motor.createUniversalMotor
import kotlin.math.absoluteValue
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.tan
import org.team5987.annotation.command_enum.CommandEnumSetTarget
import org.wpilib.command3.Command
import org.wpilib.command3.Mechanism
import org.wpilib.command3.Trigger
import org.wpilib.units.measure.Distance
import org.wpilib.units.measure.Voltage

object Elevator : Mechanism(), ElevatorHeightsCommandFactory, SysIdable {
    private val mainMotor =
        createUniversalMotor(
            MAIN_PORT,
            config = MOTOR_CONFIG,
            gearRatio = GEAR_RATIO,
            simGains = SIM_GAINS,
            linearSystemWheelDiameter = DIAMETER,
        )
    private val auxMotor =
        createUniversalMotor(
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

    var targetOffset = getGridOffset(TowerXOffset.HIGH)

    val distance by periodic { (drive.pose.x - targetOffset).absoluteValue.m }

    val targetLength: Distance by periodic {
        distance * tan(ELEVATOR_ANGLE[rad])
    }

    val targetHeight: Distance by periodic {
        distance / cos(ELEVATOR_ANGLE[rad])
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
                targetOffset = getGridOffset(TowerXOffset.LOW)
                setpoint = MIN_LENGTH
                mainMotor.setControl(
                    torqueCurrentFOC with
                        MIN_LENGTH.toAngle(DIAMETER, GEAR_RATIO)
                )
                atSetpoint.waitUntil()
            }
            .named("close")

    @CommandEnumSetTarget
    override fun setTarget(value: ElevatorHeights): UnnamedCommand = this {
        while (true) {
            if (targetHeight >= value.minHeight) {
                targetOffset = getGridOffset(value.towerXOffset)
                setpoint = targetLength
                mainMotor.setControl(
                    torqueCurrentFOC with
                        targetLength.toAngle(DIAMETER, GEAR_RATIO)
                )
            }
            yield()
        }
    }

    override fun setVoltage(voltage: Voltage) = mainMotor.setVoltage(voltage)

    override val sysidConfig =
        SysIdMechanismConfig(
            SysIdRoutineConfig(
                1.volts.per(sec),
                2.volts,
                5.sec,
                SysIdRoutine.Direction.FORWARD,
            ),
            SysIdRoutineConfig(
                1.volts.per(sec),
                2.volts,
                5.sec,
                SysIdRoutine.Direction.REVERSE,
            ),
        )
}
