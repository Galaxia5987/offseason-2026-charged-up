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
import frc.robot.lib.getTranslation3d
import frc.robot.subsystems.roller.DispatchRoller
import frc.robot.subsystems.wrist.Wrist
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput
import org.wpilib.math.geometry.Pose3d
import org.wpilib.math.geometry.Translation3d
import kotlin.math.cos
import kotlin.math.sin

private val rootPosition = Translation3d(0.350, -0.363, 0.05066)
private val IntakeRoller = -Translation3d(-0.179, -0.015, -0.205) + rootPosition
private val FourBarRollerLink = -Translation3d(-0.179, -0.7, -0.205) + rootPosition
private val FourBarLowerLink = -Translation3d(0.063, -0.708, -0.027) + rootPosition
private val FourBarUpperLink = -Translation3d(0.193, -0.708, -0.052) + rootPosition
private val GripRoller = -Translation3d(0.461, -0.482, -0.329) + rootPosition
private val Gripper = -Translation3d(0.162, -0.344, -0.394) + rootPosition
private val ConveyorRoller = -Translation3d(0.215, -0.171, -0.308) + rootPosition
private val ElevatorStage1 = -Translation3d(0.098, -0.344, -0.401) + rootPosition
private val ElevatorStage2 = -Translation3d(0.071, -0.344, -0.387) + rootPosition
private val IntakeRoller2 = -Translation3d(-0.347, -0.015, -0.07) + rootPosition
private val GripRoller2 = -Translation3d(0.461, -0.205, -0.329) + rootPosition
private val ConveyorRoller2 = -Translation3d(0.099, -0.171, -0.303) + rootPosition
private val ConveyorRoller3 = -Translation3d(0.422, -0.171, -0.291) + rootPosition
private val DispatchRollerObj = -Translation3d(0.673, -0.171, -0.257) + rootPosition
private val FourBarRollerLink2 = -Translation3d(-0.179, -0.008, -0.205) + rootPosition
private val FourBarLowerLink2 = -Translation3d(0.063, 0.008, -0.027) + rootPosition
private val FourBarUpperLink2 = -Translation3d(0.193, 0.008, -0.052) + rootPosition

private object FourBar {
    private val rUpperLinkage = 401.41682.mm
    private val upperLinkagePivotPoint = getTranslation3d(0.17, -0.358, 0.1)
    val upperLinkage by periodic {
        getPose3d(FourBarUpperLink2).rotateAround(upperLinkagePivotPoint, (-Wrist.inputs.position).toPitch())
    }
    private val theta0 = 23.945232.deg
    val rollerLink by periodic {
        getPose3d(
            cos((Wrist.inputs.position + theta0)[rad]) * rUpperLinkage[m] + upperLinkagePivotPoint.x,
            upperLinkagePivotPoint.y, sin((Wrist.inputs.position + theta0)[rad]) * rUpperLinkage[m] + upperLinkagePivotPoint.z
        )
    }

}

private val subsystemPoseArray = Array(17) { Pose3d() }

@LoggedOutput(key = "Visualization/mechanismPoses", level = LogLevel.COMP)
val mechanismPoses by periodic {
    subsystemPoseArray[0] = getPose3d(IntakeRoller)
    subsystemPoseArray[1] = getPose3d(FourBarRollerLink)
    subsystemPoseArray[2] = getPose3d(FourBarLowerLink)
    subsystemPoseArray[3] = getPose3d(FourBarUpperLink)
    subsystemPoseArray[4] = getPose3d(GripRoller)
    subsystemPoseArray[5] = getPose3d(Gripper)
    subsystemPoseArray[6] = getPose3d(ConveyorRoller)
    subsystemPoseArray[7] = getPose3d(ElevatorStage1)
    subsystemPoseArray[8] = getPose3d(ElevatorStage2)
    subsystemPoseArray[9] = getPose3d(IntakeRoller2)
    subsystemPoseArray[10] = getPose3d(GripRoller2)
    subsystemPoseArray[11] = getPose3d(ConveyorRoller2)
    subsystemPoseArray[12] = getPose3d(ConveyorRoller3)
    subsystemPoseArray[13] = getPose3d(DispatchRollerObj)
    subsystemPoseArray[14] = FourBar.rollerLink
    subsystemPoseArray[15] = getPose3d(FourBarLowerLink2)
//    subsystemPoseArray[16] = getPose3d(FourBarUpperLink2)
    subsystemPoseArray[16] = FourBar.upperLinkage
    subsystemPoseArray
}

@LoggedOutput(key = "Visualization/zeroedPoses", level = LogLevel.COMP)
val zeroedPoses = Array(17) { Pose3d() }

@LoggedOutput(key = "Visualization/zeroedPose", level = LogLevel.COMP)
val zeroedPose = Pose3d()