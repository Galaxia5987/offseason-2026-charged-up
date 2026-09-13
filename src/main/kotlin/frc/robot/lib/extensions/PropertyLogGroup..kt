package frc.robot.lib.extensions

import frc.robot.lib.getFileNameFromStack
import kotlin.reflect.KProperty0

class PropertyLogGroup(
    vararg properties: KProperty0<*>,
    loggingPath: String = "Subsystems/" + getFileNameFromStack(),
) {
    private val recordedProperties = properties.map { property ->
        "$loggingPath/${property.name}" to property
    }

    fun log() {
        recordedProperties.forEach { (key, property) ->
            property.get()?.log(key)
        }
    }
}
