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
    YELLOW(Color.YELLOW),
    GREEN(Color.GREEN),
    NONE(null)
}

fun getGridOffset(towerXOffset: TowerXOffset) =
    Translation2d(towerXOffset.offset[m], 0.0).flipIfNeeded().x

val SCORING_POSTS_START_OFFSET = 0.m

val SCORING_POSTS_X = 1.m

val SCORING_POST_GAP = 5.cm

val SCORING_POST_WIDTH = 1.m

val NUM_POSTS = 9

val SCORING_POSTS = Array(NUM_POSTS) { index ->
    Translation2d(SCORING_POSTS_X, SCORING_POSTS_START_OFFSET + (index * (SCORING_POST_WIDTH + SCORING_POST_GAP)[m]).m)
}