package frc.robot.field

import frc.robot.lib.IS_RED
import frc.robot.lib.extensions.*
import frc.robot.lib.flipper
import frc.robot.lib.getPose2d
import org.wpilib.units.measure.Distance
import org.wpilib.util.Color

val CUBE_SIZE = 24.13.cm
val PLATFORM_SIZE = 143.cm

enum class TowerXOffset(val offset: Distance) {
    HIGH(PLATFORM_SIZE - 101.cm),
    MID(PLATFORM_SIZE - 58.cm),
    LOW(PLATFORM_SIZE - CUBE_SIZE / 2),
}

enum class CubeColors(val color: Color) {
    RED(Color.RED),
    YELLOW(Color.YELLOW),
    GREEN(Color.GREEN),
    NONE(Color.BLACK)
}

fun getGridOffset(towerXOffset: TowerXOffset) =
    towerXOffset.offset[cm] * if (IS_RED) -1 else 1

private val SCORING_POSTS_START_OFFSET = 0.m

private val SCORING_POSTS_X = 1.m

private val SCORING_POST_GAP = 5.cm

private val SCORING_POST_WIDTH = 1.m

private const val NUM_POSTS = 9

val SCORING_POSTS = { isRed: Boolean ->
    List(NUM_POSTS) { index ->
        getPose2d(
            SCORING_POSTS_X,
            SCORING_POSTS_START_OFFSET +
                    (index * (SCORING_POST_WIDTH + SCORING_POST_GAP)[m]).m
        ).flip(isRed)
    }
}.flipper()
