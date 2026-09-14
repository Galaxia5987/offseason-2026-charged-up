package frc.robot.field

import frc.robot.lib.extensions.*
import org.wpilib.math.geometry.Translation2d
import org.wpilib.units.measure.Distance
import java.awt.Color

val CUBE_SIZE = 24.13.cm
val PLATFORM_SIZE = 143.cm

enum class TowerXOffset(val offset: Distance) {
    HIGH(PLATFORM_SIZE - 101.cm),
    MID(PLATFORM_SIZE - 58.cm),
    LOW(PLATFORM_SIZE - CUBE_SIZE / 2),
}

enum class CubeColors(val color: Color?) {
    RED(Color.RED),
    YELLO(Color.YELLOW),
    GREEN(Color.GREEN),
    NONE(null)
}

fun getGridOffset(towerXOffset: TowerXOffset) =
    Translation2d(towerXOffset.offset[m], 0.0).flipIfNeeded().x
