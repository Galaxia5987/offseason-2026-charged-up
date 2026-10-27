package frc.robot.subsystems.sensors

import com.ctre.phoenix6.configs.CANrangeConfiguration
import frc.robot.color_vision.ColorVision
import frc.robot.field.CubeColors
import frc.robot.lib.unified_canrange.UnifiedCANRange

interface ColorSensor {
    val color: CubeColors
}

interface DistanceSensor {
    val isPresent: Boolean
}

interface DistanceColorSensor : DistanceSensor, ColorSensor

object Sensors {
    private val IntakeCanRange =
        UnifiedCANRange(
            INTAKE_SENSOR_CANRANGE_PORT,
            configuration = CANrangeConfiguration(),
        )
    private val GripCanRange =
        UnifiedCANRange(
            GRIP_CANRANGE_PORT,
            configuration = CANrangeConfiguration(),
        )

    val IntakeSensor =
        object : DistanceSensor {
            override val isPresent: Boolean
                get() = IntakeCanRange.isInRange
        }

    val BodySensor: DistanceColorSensor =
        ColorVision("SynapseRPI", "BodySensor")

    val DispatchSensor = ColorVision("SynapseRPI", "DispatchSensor")

    val GripSensor: DistanceColorSensor =
        object : ColorVision("SynapseRPI", "DispatchSensor") {
            override val isPresent: Boolean
                get() = GripCanRange.isInRange
        }
}
