package frc.robot.color_vision

import frc.robot.field.CubeColors
import frc.robot.subsystems.sensors.DistanceColorSensor
import org.wpilib.networktables.NetworkTableInstance

const val DATA_PATH = "data"

open class ColorVision(
    synapseInstance: String,
    cameraName: String = "C270 HD WEBCAM",
) : DistanceColorSensor, AutoCloseable {
    private val table =
        NetworkTableInstance.getDefault()
            .getTable(synapseInstance)
            .getSubTable(cameraName)
            .getSubTable(DATA_PATH)
    private val green =
        table
            .getBooleanTopic(CubeColors.GREEN.networkTableName)
            .subscribe(false)
    private val yellow =
        table
            .getBooleanTopic(CubeColors.YELLOW.networkTableName)
            .subscribe(false)
    private val red =
        table.getBooleanTopic(CubeColors.RED.networkTableName).subscribe(false)

    override val color: CubeColors
        get() =
            when {
                green.get() -> CubeColors.GREEN
                yellow.get() -> CubeColors.YELLOW
                red.get() -> CubeColors.RED
                else -> CubeColors.NONE
            }

    override fun close() {
        green.close()
        yellow.close()
        red.close()
    }

    override val isPresent: Boolean
        get() = color != CubeColors.NONE
}
