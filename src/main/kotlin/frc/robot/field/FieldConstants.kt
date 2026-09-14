package frc.robot.field

import frc.robot.lib.extensions.*
import org.wpilib.math.geometry.Translation2d
import org.wpilib.units.measure.Distance

val CUBE_SIZE = 24.13.cm
val platformSize = 143.cm

enum class TowerXOffset(val offset: Distance) {
    HIGH(platformSize - 101.cm),
    MID(platformSize - 58.cm),
    LOW(platformSize - CUBE_SIZE / 2),
}

fun getGridOffset(towerXOffset: TowerXOffset) =
    Translation2d(towerXOffset.offset[m], 0.0).flipIfNeeded().x
