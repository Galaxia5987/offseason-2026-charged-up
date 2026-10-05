package frc.robot.field

import frc.robot.drive
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.flip
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.periodic
import frc.robot.lib.extensions.toYaw
import frc.robot.lib.flipper
import frc.robot.lib.getPose3d
import frc.robot.lib.getRotation3d
import frc.robot.lib.getTranslation3d
import frc.robot.lib.logged_output.LoggedOutputManager
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput
import org.wpilib.math.geometry.Pose2d
import org.wpilib.util.Color

enum class CubeColors(val color: Color) {
    RED(Color.RED),
    YELLOW(Color.YELLOW),
    GREEN(Color.GREEN),
    NONE(Color.BLACK),
}

private val SCORING_POSTS_START_OFFSET = 0.527.m

private val SCORING_POST_Y_GAP = 0.556.m

private const val NUM_POSTS = 8

private val POST_ORIGIN = getTranslation3d(1.147, 0.844, 0.0)
private val LEVEL_2_ORIGIN = POST_ORIGIN + getTranslation3d(-0.213, 0.252, 0.543)
private val LEVEL_3_ORIGIN = POST_ORIGIN + getTranslation3d(-0.667, 0.252, 0.852)

val SCORING_POSTS by lazy {
    { isRed: Boolean ->
        List(3) { level ->
            List(NUM_POSTS) { index ->
                when (level) {
                    0 -> getPose3d(
                        2.0,
                        SCORING_POSTS_START_OFFSET[m] +
                                (index * (SCORING_POST_Y_GAP)[m] + SCORING_POST_Y_GAP[m]),
                        0.0
                    , if (isRed) 180.deg.toYaw() else 0.deg.toYaw()
                    )

                    1 -> getPose3d(
                        LEVEL_2_ORIGIN.x,
                        LEVEL_2_ORIGIN.y + (index * (SCORING_POST_Y_GAP)[m]),
                        LEVEL_2_ORIGIN.z
                        , if (isRed) 180.deg.toYaw() else 0.deg.toYaw()
                    )

                    2 -> getPose3d(
                        LEVEL_3_ORIGIN.x,
                        LEVEL_3_ORIGIN.y + (index * (SCORING_POST_Y_GAP)[m]),
                        LEVEL_3_ORIGIN.z
                        , if (isRed) 180.deg.toYaw() else 0.deg.toYaw()
                    )

                    else -> error("Unknown level $level")
                }
                    .flip(isRed)
            }
        }
    }
        .flipper().also {
            LoggedOutputManager.register("FieldConstants/ScoringPosts", LogLevel.COMP) {
                it.get().flatten().toTypedArray()
            }
        }
}

fun getPostByLevel(level: Int) = SCORING_POSTS.get()[level][0]

@LoggedOutput(LogLevel.COMP, "nearestPost", "FieldConstants")
val nearestPost: Pose2d by periodic {
    drive.pose.nearest(SCORING_POSTS.get()[0].map { it.toPose2d() })
}