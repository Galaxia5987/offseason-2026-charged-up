package frc.robot.lib

import org.wpilib.math.controller.PIDController

class TunablePIDController(
    val name: String,
    kp: Double,
    ki: Double,
    kd: Double,
) : PIDController(kp, ki, kd) {

    private val loggedGains = LoggedNetworkGains(name, kp, ki, kd)

    fun update() {
        if (loggedGains.updatePIDGains()) {
            setPID(
                loggedGains.kP.value,
                loggedGains.kI.value,
                loggedGains.kD.value,
            )
        }
    }
}
