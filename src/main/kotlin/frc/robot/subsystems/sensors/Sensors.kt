package frc.robot.subsystems.sensors

import com.ctre.phoenix6.configs.CANrangeConfiguration
import frc.robot.field.CubeColors
import frc.robot.lib.unified_canrange.UnifiedCANRange

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

    val intakeSensor =
        object : DistanceSensor {
            override val isPresent: Boolean
                get() = intakeCanRange.isInRange
        }
    val bodySensor =
        object : DistanceColorSensor {
            override val isPresent: Boolean
                get() = false

            override val color: CubeColors
                get() = CubeColors.NONE
        }

    val dispatchSensor =
        object : DistanceColorSensor {
            override val isPresent: Boolean
                get() = false

            override val color: CubeColors
                get() = CubeColors.NONE
        }
    val gripSensor =
        object : DistanceColorSensor {
            override val isPresent: Boolean
                get() = gripCanRange.isInRange

            override val color: CubeColors
                get() = CubeColors.NONE
        }
}
