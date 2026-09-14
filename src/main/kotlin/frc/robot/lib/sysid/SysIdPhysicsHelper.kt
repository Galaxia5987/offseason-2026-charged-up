package frc.robot.lib.sysid

import frc.robot.lib.extensions.*
import kotlin.math.max
import kotlin.math.min
import org.wpilib.math.system.DCMotor
import org.wpilib.units.measure.Distance
import org.wpilib.units.measure.Mass

private const val g = 9.81
private const val BATTERY_VOLTAGE = 12.0

fun createSysIdLinearConfig(
    motor: DCMotor,
    mass: Mass,
    spoolRadius: Distance,
    gearing: Double,
    usableRange: Distance,
    isVertical: Boolean,
    timeoutSafetyMargin: Double = 0.7,
    minTimeoutSeconds: Double = 0.5,
    stepVoltageDelta: Double = 3.0,
    maxStepVoltage: Double = 10.0,
    quasistaticVoltageDelta: Double = 6.0,
    minReverseVoltage: Double = 1.0,
): SysIdMechanismConfig {

    val maxVelocityMps = (motor.freeSpeed / gearing) * spoolRadius[m]
    val timeAtMaxSpeed = usableRange[m] / maxVelocityMps
    val timeoutSeconds =
        max(timeAtMaxSpeed * timeoutSafetyMargin, minTimeoutSeconds)

    val maxForceNewtons = (motor.stallTorque * gearing) / spoolRadius[m]

    var gravityVoltage = 0.0
    if (isVertical) {
        val gravityForceNewtons = mass[kg] * g
        gravityVoltage =
            (gravityForceNewtons / maxForceNewtons) * BATTERY_VOLTAGE
    }

    val fwdStepVolts = min(gravityVoltage + stepVoltageDelta, maxStepVoltage)
    val fwdTargetQuasistaticVolts =
        min(gravityVoltage + quasistaticVoltageDelta, BATTERY_VOLTAGE)
    val fwdRampRate = fwdTargetQuasistaticVolts / timeoutSeconds

    val bwdStepVolts =
        min(
            max(stepVoltageDelta - gravityVoltage, minReverseVoltage),
            maxStepVoltage,
        )
    val bwdTargetQuasistaticVolts =
        min(
            max(quasistaticVoltageDelta - gravityVoltage, minReverseVoltage),
            BATTERY_VOLTAGE,
        )
    val bwdRampRate = bwdTargetQuasistaticVolts / timeoutSeconds

    return SysIdMechanismConfig(
        forward =
            SysIdRoutineConfig(
                rampRate = fwdRampRate.volts.per(sec),
                stepVoltage = fwdStepVolts.volts,
                timeout = timeoutSeconds.sec,
                direction = SysIdRoutine.Direction.FORWARD,
            ),
        backward =
            SysIdRoutineConfig(
                rampRate = bwdRampRate.volts.per(sec),
                stepVoltage = bwdStepVolts.volts,
                timeout = timeoutSeconds.sec,
                direction = SysIdRoutine.Direction.REVERSE,
            ),
    )
}
