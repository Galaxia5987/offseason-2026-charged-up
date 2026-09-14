package frc.robot.field

import frc.robot.lib.extensions.cm
import frc.robot.lib.extensions.flipIfNeeded
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.periodic
import org.wpilib.math.geometry.Translation2d
import org.wpilib.units.measure.Distance

val CUBE_SIZE = 24.13.cm
val TOWER_Y_OFFSET: Distance = CUBE_SIZE + 0.1.m
val FIRST_TOWER_LOCATION = Translation2d(1.m, TOWER_Y_OFFSET)
val TOWER_GRID by periodic {
    Translation2d(
        FIRST_TOWER_LOCATION.x, 0.0
    ).flipIfNeeded()
}
