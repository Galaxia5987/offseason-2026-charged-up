package frc.robot.field

import frc.robot.lib.extensions.flipIfNeeded
import frc.robot.lib.extensions.m
import frc.robot.lib.getTranslation2d

val towerPoseYOffset = 0.2.m
val towerTranslation = getTranslation2d(1.0.m, towerPoseYOffset).flipIfNeeded()
