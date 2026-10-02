package frc.robot.subsystems.sensors

import com.ctre.phoenix6.configs.CANrangeConfiguration
import frc.robot.CURRENT_MODE
import frc.robot.field.CubeColors
import frc.robot.lib.Mode
import frc.robot.lib.unified_canrange.UnifiedCANRange
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean

interface ColorSensor {
    val color: CubeColors
    val isGreen: Boolean
        get() = (color == CubeColors.GREEN)

    val isYellow: Boolean
        get() = (color == CubeColors.YELLOW)

    val isRed: Boolean
        get() = (color == CubeColors.RED)
}

interface DistanceSensor {
    val isPresent: Boolean
}

interface DistanceColorSensor : DistanceSensor, ColorSensor

fun <T> switchIfSim(real: T, sim: T): T = when(CURRENT_MODE) {
    Mode.REAL -> real
    Mode.SIM -> sim
    else -> sim
}

fun makeSimDistanceSensor(name: String): DistanceSensor = object : DistanceSensor {
    private val loggedIsPresent = LoggedNetworkBoolean("Tuning/$name/isPresent", false)
    override val isPresent: Boolean
        get() = loggedIsPresent.get()
}

fun makeSimDistanceColorSensor(name: String): DistanceColorSensor = object : DistanceColorSensor {
    private val loggedIsGreen = LoggedNetworkBoolean("Tuning/$name/colors/green", false)
    private val loggedIsYellow = LoggedNetworkBoolean("Tuning/$name/colors/yellow", false)
    private val loggedIsRed = LoggedNetworkBoolean("Tuning/$name/colors/red", false)

    override val isPresent: Boolean
        get() = loggedIsRed.get() || loggedIsYellow.get() || loggedIsGreen.get()
    override val color: CubeColors
        get() = when{
            loggedIsGreen.get() -> CubeColors.GREEN
            loggedIsYellow.get() -> CubeColors.YELLOW
            loggedIsRed.get() -> CubeColors.RED
            else -> CubeColors.NONE
        }
}

object Sensors {
    private val intakeCanRange =
        UnifiedCANRange(
            INTAKE_SENSOR_CANRANGE_PORT,
            configuration = CANrangeConfiguration(),
        )
    private val gripCanRange =
        UnifiedCANRange(
            GRIP_CANRANGE_PORT,
            configuration = CANrangeConfiguration(),
        )

    val intakeSensor = switchIfSim(
        object : DistanceSensor {
            override val isPresent: Boolean
                get() = intakeCanRange.isInRange
        }, makeSimDistanceSensor("intakeSensor"))
    val bodySensor = switchIfSim(
        object : DistanceColorSensor {
            override val isPresent: Boolean
                get() = false

            override val color: CubeColors
                get() = CubeColors.NONE
        }, makeSimDistanceColorSensor("bodySensor"))

    val dispatchSensor = switchIfSim(
        object : DistanceColorSensor {
            override val isPresent: Boolean
                get() = false

            override val color: CubeColors
                get() = CubeColors.NONE
        }, makeSimDistanceColorSensor("dispatchSensor"))
    val gripSensor = switchIfSim(
        object : DistanceColorSensor {
            override val isPresent: Boolean
                get() = gripCanRange.isInRange

            override val color: CubeColors
                get() = CubeColors.NONE
        }, makeSimDistanceColorSensor("gripSensor"))
}
