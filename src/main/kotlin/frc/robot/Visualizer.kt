package frc.robot

import frc.robot.lib.extensions.mm
import frc.robot.lib.extensions.periodic
import frc.robot.lib.extensions.toRoll
import frc.robot.lib.getPose3d
import frc.robot.lib.getTranslation3d
import frc.robot.subsystems.wrist.Wrist
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput
import org.wpilib.math.geometry.Pose3d
import org.wpilib.math.geometry.Translation3d

private val rootPosition = Translation3d(0.350, -0.363, 0.05066)
private val IntakeRoller = Translation3d(-0.179, -0.015, -0.205) + rootPosition
private val FourbarRollerLink = Translation3d(-0.179, -0.7, -0.205) + rootPosition
private val FourbarLowerLink = Translation3d(0.063, -0.708, -0.027) + rootPosition
private val FourBarUpperLink = Translation3d(0.193, -0.708, -0.052) + rootPosition
private val GripRoller = Translation3d(0.461, -0.482, -0.329) + rootPosition
private val Gripper = Translation3d(0.162, -0.344, -0.394) + rootPosition
private val ConveyorRoller = Translation3d(0.215, -0.171, -0.308) + rootPosition
private val ElevatorStage1 = Translation3d(0.098, -0.344, -0.401) + rootPosition
private val ElevatorStage2 = Translation3d(0.071, -0.344, -0.387) + rootPosition
private val IntakeRoller2 = Translation3d(-0.347, -0.015, -0.07) + rootPosition
private val GripRoller2 = Translation3d(0.461, -0.205, -0.329) + rootPosition
private val ConveyorRoller2 = Translation3d(0.099, -0.171, -0.303) + rootPosition
private val ConveyorRoller3 = Translation3d(0.422, -0.171, -0.291) + rootPosition
private val DispatchRoller = Translation3d(0.673, -0.171, -0.257) + rootPosition
private val FourBarRollerLink = Translation3d(-0.179, -0.008, -0.205) + rootPosition
private val FourBarLowerLink = Translation3d(0.063, 0.008, -0.027) + rootPosition
private val FourBarUpperLink2 = Translation3d(0.193, 0.008, -0.052) + rootPosition

private object FourBar {
    private val rUpperLinkage = 401.41682.mm
    private val pivotPointUpperLinkage = getTranslation3d(y = 210.94806.mm, z = 150.70430.mm, x = 8.00000.mm)
    val upperLinkage by periodic {
        getPose3d(FourBarUpperLink2, (-Wrist.inputs.position).toRoll())
    }
    val rollerLink by periodic {
        val angle = Wrist.inputs.position
//        getPose3d(y = rUpperLinkage-(rUpperLinkage * cos(angle[rad])), z = rUpperLinkage * sin(angle[rad]))
    }

}

private val subsystemPoseArray = Array(17) { Pose3d() }

@LoggedOutput(key = "Visualization/mechanismPoses", level = LogLevel.COMP)
val mechanismPoses by periodic {
    subsystemPoseArray[16] = FourBar.upperLinkage
//    subsystemPoseArray[8] = getPose3d(-0.071 + 0.350, 0.344 - 0.363, 0.387 + 0.05066)
    subsystemPoseArray
}

@LoggedOutput(key = "Visualization/zeroedPoses", level = LogLevel.COMP)
val zeroedPoses = Array(17) { Pose3d() }

@LoggedOutput(key = "Visualization/zeroedPose", level = LogLevel.COMP)
val zeroedPose = Pose3d()