package frc.robot.lib.universal_motor

import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.ControlRequest
import frc.robot.CURRENT_MODE
import frc.robot.lib.Gains
import frc.robot.lib.Mode
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.kg2m
import frc.robot.lib.extensions.m
import frc.robot.lib.getFileNameFromStack
import frc.robot.lib.unit_test.allMotorsFromPorts
import org.littletonrobotics.junction.Logger
import org.wpilib.units.measure.Angle
import org.wpilib.units.measure.Distance
import org.wpilib.units.measure.MomentOfInertia

/**
 * Represents a universal wrapper for a motor, which abstracts the real and
 * simulated implementations.
 *
 * @param port The CAN ID of the motor controller.
 * @param canbus The CAN bus name (optional, default is the default bus).
 * @param config Configuration for the TalonFX motor controller.
 * @param momentOfInertia The moment of inertia used in simulation.
 * @param gearRatio The gear ratio between the motor and the mechanism (default
 *   is 1.0).
 * @param linearSystemWheelDiameter The diameter of the wheel or spool for
 *   linear mechanisms.
 *
 * ```
 *        Do Not pass this parameter if the motor does not actuate a linear mechanism (e.g., elevator or a linear intake).
 * ```
 */
class UniversalTalonFX(
    val port: Int,
    private val canbus: CANBus = CANBus("rio"),
    private val subsystem: String = getFileNameFromStack(),
    private val motorName: String = "motorId $port",
    private val config: TalonFXConfiguration = TalonFXConfiguration(),
    private val simGains: Gains = Gains(1.0),
    private val momentOfInertia: MomentOfInertia = 0.003.kg2m,
    private val gearRatio: Double = 1.0,
    private val linearSystemWheelDiameter: Distance = 0.m,
    private val absoluteEncoderOffset: Angle = 0.deg,
    private val logConfig: MotorLogConfig = MotorLogConfig(),
) {
    init {
        if (CURRENT_MODE == Mode.SIM) {
            allMotorsFromPorts[port to canbus.name] = this
        }
    }

    private val motorIO: MotorIO =
        if (CURRENT_MODE == Mode.REAL)
            MotorIOReal(
                motorName,
                subsystem,
                port,
                canbus,
                config,
                gearRatio,
                linearSystemWheelDiameter,
                absoluteEncoderOffset,
                logConfig,
            )
        else {
            MotorIOSim(
                motorName,
                subsystem,
                momentOfInertia,
                config,
                simGains,
                gearRatio,
                linearSystemWheelDiameter,
                logConfig,
            )
        }
    val inputs: LoggedMotorInputs = motorIO.inputs

    /**
     * Sends a control request to the motor (e.g., voltage, velocity, position).
     *
     * @param control The control request to apply.
     */
    fun setControl(control: ControlRequest) = motorIO.setRequest(control)

    fun reset(angle: Angle = 0.deg) = motorIO.resetInternalEncoder(angle)

    fun periodic() {
        motorIO.updateInputs()
        Logger.processInputs("Subsystems/$subsystem/$motorName", inputs)
    }

    fun resetInputs() {
        motorIO.resetInputs()
    }

    fun applyConfiguration(config: TalonFXConfiguration) {
        motorIO.applyConfiguration(config)
    }
}
