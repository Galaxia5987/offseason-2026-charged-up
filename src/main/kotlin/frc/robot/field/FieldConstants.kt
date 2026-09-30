package frc.robot.field

import frc.robot.lib.IS_RED
import frc.robot.lib.extensions.*
import frc.robot.lib.flipSignOnTrue
import frc.robot.lib.flipper
import frc.robot.lib.getPose2d
import frc.robot.lib.getPose3d
import frc.robot.lib.getTranslation2d
import frc.robot.lib.getTranslation3d
import frc.robot.lib.logged_output.LoggedOutputManager
import org.team5987.annotation.LogLevel
import org.wpilib.math.geometry.Pose2d
import org.wpilib.units.measure.Distance
import org.wpilib.util.Color
import kotlin.collections.List

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
    NONE(Color.BLACK),
}

fun getGridOffset(towerXOffset: TowerXOffset) =
    towerXOffset.offset[cm] * if (IS_RED) -1 else 1

private val SCORING_POSTS_START_OFFSET = 0.527.m

private val SCORING_POST_X_GAP = 0.532.m
private val SCORING_POST_Y_GAP = 0.556.m
private val SCORING_POST_Z_GAP = 0.532.m

private val SCORING_POST_WIDTH = 1.m

private const val NUM_POSTS = 8

private val POST_ORIGIN = getTranslation3d(1.147, 0.844, 0.0)
private val LEVEL_2_ORIGIN = POST_ORIGIN + getTranslation3d(-0.213, 0.252, 0.543)
private val LEVEL_3_ORIGIN = POST_ORIGIN + getTranslation3d(-0.667, 0.252, 0.852)

val SCORING_POSTS =
    { isRed: Boolean ->
        List(3) { level ->
            List(NUM_POSTS) { index ->
                when(level) {
                    0 -> getPose3d(
                        1.193,
                        SCORING_POSTS_START_OFFSET[m] +
                                (index * (SCORING_POST_Y_GAP)[m] + SCORING_POST_Y_GAP[m]),
                        0.0
                    )
                    1 -> getPose3d(
                        LEVEL_2_ORIGIN.x,
                        LEVEL_2_ORIGIN.y + (index * (SCORING_POST_Y_GAP)[m]),
                        LEVEL_2_ORIGIN.z
                    )
                    2 -> getPose3d(
                        LEVEL_3_ORIGIN.x,
                        LEVEL_3_ORIGIN.y + (index * (SCORING_POST_Y_GAP)[m]),
                        LEVEL_3_ORIGIN.z
                    )
                    else -> error("Unknown level $level")
                }
                    .flip(isRed)
            }
        }
    }
        .flipper().also {
            LoggedOutputManager.register("FieldConstants/ScoringPosts", LogLevel.COMP) { it.get().flatten().toTypedArray() }
        }
