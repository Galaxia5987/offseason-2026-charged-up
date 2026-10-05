package frc.robot.subsystems.elevator

import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC
import com.ctre.phoenix6.signals.MotorAlignmentValue
import frc.robot.drive
import frc.robot.field.getPostByLevel
import frc.robot.lib.commands.UnnamedCommand
import frc.robot.lib.commands.addPeriodic
import frc.robot.lib.commands.invoke
import frc.robot.lib.commands.unaryPlus
import frc.robot.lib.commands.waitUntil
import frc.robot.lib.extensions.*
import frc.robot.lib.sysid.SysIdMechanismConfig
import frc.robot.lib.sysid.SysIdRoutine
import frc.robot.lib.sysid.SysIdRoutineConfig
import frc.robot.lib.sysid.SysIdable
import frc.robot.lib.universal_motor.MotorLogConfig
import frc.robot.lib.universal_motor.createUniversalMotor
import kotlin.math.absoluteValue
import kotlin.math.cos
import kotlin.math.sin
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
            logConfig = MotorLogConfig(

            )
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

    val currentHeight: Distance
        get() = mainMotor.inputs.distance * sin(ELEVATOR_ANGLE[rad]) + BASE_HEIGHT


    val inputs
        get() = mainMotor.inputs

    val distance
        get() = (drive.pose.x - getPostByLevel(namedSetpoint.level).x).absoluteValue.m

    val setLength: Distance
        get() = distance / cos(ELEVATOR_ANGLE[rad])

    val setHeight: Distance
        get() = setLength * sin(ELEVATOR_ANGLE[rad]) + BASE_HEIGHT

    val targetHeight: Distance
        get() = getPostByLevel(namedSetpoint.level).z.m


    private var setpoint = 0.m
    private var namedSetpoint = ElevatorHeights.CLOSE
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
            ::namedSetpoint,
            ::setLength,
            ::currentHeight,
            ::targetHeight,
            ::setHeight
        )

    fun periodic() {
        mainMotor.periodic()
        auxMotor.periodic()
        logList.periodic()
    }

    private fun closed(): Command =
        this {
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
        namedSetpoint = value
        val length = value.defaultLength
        if (value == ElevatorHeights.CLOSE) {
            +closed()
        } else {
            while (true) {
                if (targetHeight.isNear(setHeight, HEIGHT_TOLERANCE)) {
                    if (targetHeight <= setHeight) {
                        setLength(setLength)
                    } // TODO: Issue a warning
                } else setLength(length)
                yield()
            }
        }
    }

    private fun setLength(length: Distance) {
        setpoint = length
        mainMotor.setControl(
            torqueCurrentFOC with
                    length.toAngle(DIAMETER, GEAR_RATIO)
        )
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
