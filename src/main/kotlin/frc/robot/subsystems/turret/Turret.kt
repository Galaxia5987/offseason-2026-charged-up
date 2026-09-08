package frc.robot.subsystems.turret

import com.ctre.phoenix6.CANBus.systemcore
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.CANcoder
import frc.robot.lib.commands.addPeriodic
import frc.robot.lib.commands.supplierCommand
import frc.robot.lib.commands.waitUntil
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.rot
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger
import org.wpilib.command3.Mechanism
import org.wpilib.command3.Trigger
import org.wpilib.units.measure.Angle

object Turret : Mechanism() {
    private val absoluteEncoder = CANcoder(ENCODER_ID, systemcore(3))
    private val motor: UniversalTalonFX =
        UniversalTalonFX(
            port = PORT,
            config = CONFIG,
            gearRatio = RATIO,
            simGains = SIM_GAINS,
        )

    var setpoint: Angle = 0.deg
        private set

    val positionVoltage = PositionVoltage(0.0)
    val atSetpoint = Trigger {
        motor.inputs.position.isNear(setpoint, TOLERANCE)
    }

    val motorPosition: Angle
        get() = motor.inputs.position

    val setAngle =
        supplierCommand<Angle> { angle ->
                setpoint = constraintTurretLimit(angle)
                motor.setControl(positionVoltage.withPosition(setpoint))

                whenOneShot {
                    atSetpoint.waitUntil()
                }
            }
            .named("Subsystem/Turret/setAngle")

    fun constraintTurretLimit(angle: Angle): Angle {
        if (angle < REVERSE_LIMIT) return 1.rot + angle
        return angle
    }

    init {
        absoluteEncoder.configurator.apply(ENCODER_CONFIG)
        addPeriodic(::periodic)
    }

    fun periodic() {
        motor.periodic()
        Logger.recordOutput("Subsystems/Turret/setpoint", setpoint)
        Logger.recordOutput("Subsystems/Turret/atSetpoint", atSetpoint)
    }
}
