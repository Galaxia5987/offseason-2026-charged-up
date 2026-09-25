package frc.robot

import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.mm
import frc.robot.lib.extensions.periodic
import frc.robot.lib.extensions.rad
import frc.robot.lib.extensions.toPitch
import frc.robot.lib.extensions.toRoll
import frc.robot.lib.getPose3d
import frc.robot.lib.getRotation3d
import frc.robot.lib.getTranslation3d
import frc.robot.subsystems.wrist.Wrist
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput
import org.wpilib.math.geometry.Pose3d
import kotlin.math.cos
import kotlin.math.sin

private object FourBar {
    private val rUpperLinkage = 401.41682.mm
    private val pivotPointUpperLinkage = getTranslation3d(y = 210.94806.mm, z = 150.70430.mm, x=8.00000.mm)
    val upperLinkage by periodic {
        getPose3d().rotateAround(pivotPointUpperLinkage, (-Wrist.inputs.position).toRoll())
    }
    val rollerLink by periodic {
        val angle = Wrist.inputs.position
        getPose3d(y = rUpperLinkage-(rUpperLinkage * cos(angle[rad])), z = rUpperLinkage * sin(angle[rad]))
    }
}

private val subsystemPoseArray = Array(17) { Pose3d() }

@LoggedOutput(key = "Visualization/mechanismPoses", level = LogLevel.COMP)
val mechanismPoses by periodic {
    subsystemPoseArray[16] = FourBar.upperLinkage
    subsystemPoseArray[14] = FourBar.rollerLink
    subsystemPoseArray
}