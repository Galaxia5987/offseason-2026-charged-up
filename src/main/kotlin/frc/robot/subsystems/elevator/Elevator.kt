package frc.robot.subsystems.elevator

import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC
import com.ctre.phoenix6.signals.MotorAlignmentValue
import frc.robot.lib.commands.UnnamedCommand
import frc.robot.lib.commands.addPeriodic
import frc.robot.lib.commands.invoke
import frc.robot.lib.commands.waitUntil
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.toAngle
import frc.robot.lib.extensions.with
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger
import org.wpilib.command3.Mechanism
import org.wpilib.command3.Trigger

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
    }

    override fun setTarget(value: ElevatorHeights): UnnamedCommand = this {
        setpoint = value.toElevatorLength()
        mainMotor.setControl(
            torqueCurrentFOC with
                value.toElevatorLength().toAngle(DIAMETER, GEAR_RATIO)
        )
        isAtSetPoint.waitUntil()
    }
}
