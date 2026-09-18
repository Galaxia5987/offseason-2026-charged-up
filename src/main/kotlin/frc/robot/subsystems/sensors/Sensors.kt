package frc.robot.subsystems.sensors

import com.ctre.phoenix6.configs.CANrangeConfiguration
import frc.robot.field.CubeColors
import frc.robot.lib.unified_canrange.UnifiedCANRange
import org.wpilib.util.Color

interface Sensor {
    val isPresent: Boolean
    val color: CubeColors get() = null
}

object Sensors {
    private val IntakeCanRange = UnifiedCANRange(INTAKE_SENSOR_CANRANGE_PORT, configuration = CANrangeConfiguration())
    private val GripCanRange = UnifiedCANRange(GRIP_CANRANGE_PORT, configuration = CANrangeConfiguration())

    val IntakeSensor = object : Sensor {
        override val isPresent: Boolean
            get() = IntakeCanRange.isInRange
    }
    val bodySensor = object : Sensor {
        override val isPresent: Boolean
            get() = false
        override val color: CubeColors
            get() = CubeColors.NONE
    }

    val DispatchSensor = object : Sensor {
        override val isPresent: Boolean
            get() = false
        override val color: CubeColors
            get() = CubeColors.NONE

    }
    val GripSensor = object : Sensor {
        override val isPresent: Boolean
            get() = GripCanRange.isInRange
        override val color: CubeColors
            get() = CubeColors.NONE

    }
}