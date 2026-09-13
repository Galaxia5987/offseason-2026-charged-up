package frc.robot.field

import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.periodic
import org.wpilib.math.geometry.Translation2d

val TOWER_Y_OFFSET = 0.2.m
val FIRST_TOWER_LOCATION = Translation2d(1.m, TOWER_Y_OFFSET)
var FINISHED_TOWER_AMOUNT = 0
val TOWER_GRID by periodic {
    Translation2d(
        FIRST_TOWER_LOCATION.x,
        FIRST_TOWER_LOCATION.y + TOWER_Y_OFFSET[m] * FINISHED_TOWER_AMOUNT,
    )
}
