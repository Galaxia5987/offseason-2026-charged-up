package frc.robot.subsystems.shooter.flywheel

import com.ctre.phoenix6.CANBus.systemcore
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.signals.MotorAlignmentValue
import frc.robot.lib.commands.addPeriodic
import frc.robot.lib.commands.supplierCommand
import frc.robot.lib.commands.waitUntil
import frc.robot.lib.extensions.rps
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger
import org.wpilib.command3.Mechanism
import org.wpilib.command3.Trigger
import org.wpilib.units.measure.AngularVelocity

object Flywheel : Mechanism() {
    val mainMotor =
        UniversalTalonFX(
            canbus = systemcore(0),
            port = MAIN_MOTOR_PORT,
            config = MOTOR_CONFIG,
            gearRatio = GEAR_RATIO,
            logConfig = MOTOR_LOG_CONFIG,
        )

    val auxMotor1 =
        UniversalTalonFX(
            canbus = systemcore(0),
            port = FIRST_AUX_MOTOR_PORT,
            config = MOTOR_CONFIG,
            gearRatio = GEAR_RATIO,
            logConfig = MOTOR_LOG_CONFIG,
        )
    val auxMotor2 =
        UniversalTalonFX(
            canbus = systemcore(0),
            port = SECOND_AUX_MOTOR_PORT,
            config = MOTOR_CONFIG,
            gearRatio = GEAR_RATIO,
            logConfig = MOTOR_LOG_CONFIG,
        )

    var setpoint = 0.rps
    val atSetpoint = Trigger {
        mainMotor.inputs.velocity.isNear(setpoint, TOLERANCE)
    }

    private val velocityVoltage = VelocityVoltage(0.0)

    init {
        addPeriodic(::periodic)
        auxMotor1.setControl(
            Follower(MAIN_MOTOR_PORT, MotorAlignmentValue.Opposed)
        )
        auxMotor2.setControl(
            Follower(MAIN_MOTOR_PORT, MotorAlignmentValue.Aligned)
        )
    }

    val setVelocity =
        supplierCommand<AngularVelocity> { velocity ->
                setpoint = velocity
                mainMotor.setControl(velocityVoltage.withVelocity(velocity))

                whenOneShot {
                    atSetpoint.waitUntil()
                }
            }
            .named("Subsystems/Flywheel/setVelocity")

    fun periodic() {
        mainMotor.periodic()
        auxMotor1.periodic()
        auxMotor2.periodic()

        Logger.recordOutput("Subsystems/Flywheel/atSetpoint", atSetpoint)
        Logger.recordOutput("Subsystems/Flywheel/setpoint", setpoint)
        Logger.recordOutput(
            "Subsystems/Flywheel/setpointError",
            setpoint - mainMotor.inputs.velocity,
        )
    }
}
