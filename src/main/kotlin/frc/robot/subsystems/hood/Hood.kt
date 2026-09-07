package frc.robot.subsystems.hood

import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.CANcoder
import frc.robot.lib.commands.addPeriodic
import frc.robot.lib.commands.supplierCommand
import frc.robot.lib.commands.waitUntil
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.log
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.wpilib.command3.Mechanism
import org.wpilib.command3.Trigger
import org.wpilib.units.measure.Angle

object Hood : Mechanism() {
    private val absoluteEncoder = CANcoder(ENCODER_ID, CANBUS)
    private val motor =
        UniversalTalonFX(
            port = PORT,
            canbus = CANBUS,
            config = CONFIG,
            gearRatio = GEAR_RATIO,
            simGains = SIM_GAINS,
            absoluteEncoderOffset = ABSOLUTE_ENCODER_OFFSET.deg,
        )

    init {
        addPeriodic(::periodic)
        absoluteEncoder.configurator.apply(ENCODER_CONFIG)
    }

    private var positionRequest = PositionVoltage(0.deg)
    private var setpoint = 0.deg
    val atSetpoint = Trigger {
        motor.inputs.position.isNear(setpoint, TOLERANCE)
    }

    val setPosition =
        supplierCommand<Angle> { angle ->
                setpoint = angle
                motor.setControl(positionRequest.withPosition(setpoint))

                whenOneShot { atSetpoint.waitUntil() }
            }
            .named("Subsystems/Hood/setPosition")

    fun periodic() {
        motor.periodic()
        mapOf(
                "atSetpoint" to atSetpoint,
                "setpoint" to setpoint,
                "setpointError" to setpoint - motor.inputs.position,
            )
            .log("Subsystems/Hood")
    }
}
