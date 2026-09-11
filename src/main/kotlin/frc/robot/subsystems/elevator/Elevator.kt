package frc.robot.subsystems.elevator

import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC
import com.ctre.phoenix6.signals.MotorAlignmentValue
import frc.robot.lib.commands.UnnamedCommand
import frc.robot.lib.commands.addPeriodic
import frc.robot.lib.commands.invoke
import frc.robot.lib.extensions.*
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger
import org.wpilib.command3.Command
import org.wpilib.command3.Mechanism
import org.wpilib.command3.Trigger
import kotlin.math.sin

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

    private var setpoint = 0.m
    private val torqueCurrentFOC = MotionMagicTorqueCurrentFOC(0.0)

    val isAtSetPoint = Trigger {
        setpoint.isNear(mainMotor.inputs.distance, TOLERANCE)
    }

    init {
        addPeriodic(::periodic)
        auxMotor.setControl(
            Follower(mainMotor.port, MotorAlignmentValue.Opposed)
        )
    }

    fun periodic() {
        mainMotor.periodic()
        auxMotor.periodic()
        Logger.recordOutput("Subsystems/Elevator/isAtSetpoint", isAtSetPoint)
        Logger.recordOutput("Subsystems/Elevator/setpoint", setpoint)
        Logger.recordOutput("Subsystems/Elevator/Height", mainMotor.inputs.distance * sin(ELEVATOR_ANGLE[rad]))
    }

    fun close(): Command = this {
        setpoint = 0.0.m
        mainMotor.setControl(
            torqueCurrentFOC with 0.0
        )
    }.named("close")

    override fun setTarget(value: ElevatorHeights): UnnamedCommand = this {
        while (true) {
            if (value.calculateHeight() < value.minHeight) continue
            setpoint = value.calculateDropDistance()
            mainMotor.setControl(
                torqueCurrentFOC with
                        value.calculateDropDistance().toAngle(DIAMETER, GEAR_RATIO)
            )
            yield()
        }
    }
}
