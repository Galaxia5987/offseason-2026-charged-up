package frc.robot.colorVision

import frc.robot.field.CubeColors
import frc.robot.lib.extensions.periodic
import frc.robot.subsystems.sensors.ColorSensor
import org.wpilib.networktables.NetworkTableInstance

open class ColorVision(synapseInstance: String, cameraName: String = "C270 HD WEBCAM") : ColorSensor,
    AutoCloseable {
    private val table = NetworkTableInstance.getDefault()
        .getTable(synapseInstance)
        .getSubTable(cameraName)
        .getSubTable(DATA_PATH)
    private val green =
        table.getBooleanTopic(CubeColors.GREEN.networkTableName).subscribe(false)
    private val yellow =
        table.getBooleanTopic(CubeColors.YELLOW.networkTableName).subscribe(false)
    private val red =
        table.getBooleanTopic(CubeColors.RED.networkTableName).subscribe(false)

    override val color: CubeColors by periodic {
        val hasTable =
            mapOf(CubeColors.GREEN to green.get(), CubeColors.YELLOW to yellow.get(), CubeColors.RED to red.get())
        var shownColor = CubeColors.NONE
        hasTable.forEach {
            if (it.value) {
                if (shownColor != CubeColors.NONE) return@periodic CubeColors.NONE
                shownColor = it.key
            }
        }
        return@periodic shownColor
    }

    override fun close() {
        green.close()
        yellow.close()
        red.close()
    }

}