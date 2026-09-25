package frc.robot

import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.mm
import frc.robot.lib.extensions.periodic
import frc.robot.lib.extensions.toPitch
import frc.robot.lib.extensions.toRoll
import frc.robot.lib.getPose3d
import frc.robot.lib.getRotation3d
import frc.robot.lib.getTranslation3d
import frc.robot.subsystems.wrist.Wrist
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput
import org.wpilib.math.geometry.Pose3d

private object FourBar {
    private val rUpperLinkage = 401.41682.mm
    private val pivotPointUpperLinkage = getTranslation3d(z=193.775.mm, x= -(52.3).mm)
    val upperLinkage = getPose3d().rotateAround(pivotPointUpperLinkage, Wrist.inputs.position.toRoll())
}

private val subsystemPoseArray = Array(17) { Pose3d() }

@LoggedOutput(key = "Visualization/mechanismPoses", level = LogLevel.COMP)
val mechanismPoses by periodic {
    subsystemPoseArray[16] = FourBar.upperLinkage
    subsystemPoseArray
}