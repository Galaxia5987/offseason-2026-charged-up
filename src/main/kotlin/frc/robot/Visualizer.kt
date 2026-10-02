package frc.robot

import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.mm
import frc.robot.lib.extensions.periodic
import frc.robot.lib.extensions.rad
import frc.robot.lib.extensions.toPitch
import frc.robot.lib.extensions.toRoll
import frc.robot.lib.extensions.withRotation
import frc.robot.lib.getPose3d
import frc.robot.lib.getTranslation3d
import frc.robot.subsystems.elevator.Elevator
import frc.robot.subsystems.roller.ConveyorRoller
import frc.robot.subsystems.roller.DispatchRoller
import frc.robot.subsystems.roller.GripRoller
import frc.robot.subsystems.roller.IntakeRoller
import frc.robot.subsystems.wrist.Wrist
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput
import org.wpilib.math.geometry.Pose3d
import org.wpilib.math.geometry.Translation3d
import kotlin.math.cos
import kotlin.math.sin

private val rootPosition = Translation3d(0.350, -0.363, 0.05066)
private val IntakeRollerObj = -Translation3d(-0.179, -0.015, -0.205) + rootPosition
private val FourBarRollerLink = -Translation3d(-0.179, -0.7, -0.205) + rootPosition
private val FourBarLowerLink = -Translation3d(0.063, -0.708, -0.027) + rootPosition
private val FourBarUpperLink = -Translation3d(0.193, -0.708, -0.052) + rootPosition
private val GripRollerObj = -Translation3d(0.461, -0.482, -0.329) + rootPosition
private val Gripper = -Translation3d(0.162, -0.344, -0.394) + rootPosition
private val ConveyorRoller1 = -Translation3d(0.215, -0.171, -0.308) + rootPosition
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
    private val upperLinkagePivotPoint2 = getTranslation3d(0.17, -0.358, 0.1)
    val upperLinkage by periodic {
        getPose3d(FourBarUpperLink2).rotateAround(upperLinkagePivotPoint, (-Wrist.inputs.position).toPitch())
    }
    val upperLinkage2 by periodic {
        getPose3d(upperLinkage.translation.x, -upperLinkage.translation.y, upperLinkage.translation.z, rotation = upperLinkage.rotation)
    }
    private val theta0 = 23.945232.deg
    val rollerLink by periodic {
        getPose3d(
            cos((Wrist.inputs.position + theta0)[rad]) * rUpperLinkage[m] + upperLinkagePivotPoint.x,
            upperLinkagePivotPoint.y, sin((Wrist.inputs.position + theta0)[rad]) * rUpperLinkage[m] + upperLinkagePivotPoint.z
        )
    }
    val rollerLink2 by periodic {
        getPose3d(rollerLink.translation.x, -rollerLink.translation.y, rollerLink.translation.z)
    }

    val intakeRoller by periodic {
        rollerLink.withRotation(IntakeRoller.inputs.position.toPitch())
    }

    private val rollerToRoller = getTranslation3d(0.168, 0.0, -0.135)

    val intakeRoller2 by periodic {
        getPose3d(intakeRoller.translation + rollerToRoller).withRotation(IntakeRoller.inputs.position.toPitch())
    }

}

private object ConveyorRollers {
    val conveyorRoller by periodic {
        getPose3d(ConveyorRoller1, ConveyorRoller.inputs.position.toPitch())
    }
    val conveyorRoller2 by periodic {
        getPose3d(ConveyorRoller2, rotation = conveyorRoller.rotation)
    }
    val conveyorRoller3 by periodic {
        getPose3d(ConveyorRoller3, rotation = conveyorRoller.rotation)
    }
    val dispatchRoller by periodic {
        getPose3d(DispatchRollerObj, rotation = DispatchRoller.inputs.position.toPitch())
    }
}

private object ElevatorObjects {
    private val theta0 = 27.132149.deg
    val gripper by periodic {
        getPose3d(Gripper.x - Elevator.inputs.distance[m]*cos(theta0[rad]), Gripper.y, Gripper.z + Elevator.inputs.distance[m]*sin(theta0[rad]))
    }
    val gripRoller by periodic {
        getPose3d(GripRollerObj, rotation = GripRoller.inputs.position.toRoll())
    }
    val gripRoller2 by periodic {
        getPose3d(GripRoller2, rotation = (-GripRoller.inputs.position).toRoll())
    }
}

private val subsystemPoseArray = Array(17) { Pose3d() }

@LoggedOutput(key = "Visualization/mechanismPoses", level = LogLevel.COMP)
val mechanismPoses by periodic {
    subsystemPoseArray[0] = FourBar.intakeRoller
    subsystemPoseArray[1] = FourBar.rollerLink2
    subsystemPoseArray[3] = ElevatorObjects.gripRoller
    subsystemPoseArray[2] = FourBar.upperLinkage2
    subsystemPoseArray[4] = ElevatorObjects.gripper
    subsystemPoseArray[5] = ConveyorRollers.conveyorRoller
    subsystemPoseArray[6] = getPose3d(ElevatorStage1)
    subsystemPoseArray[7] = getPose3d(ElevatorStage2)
    subsystemPoseArray[8] = FourBar.intakeRoller2
    subsystemPoseArray[9] = ElevatorObjects.gripRoller2
    subsystemPoseArray[10] = ConveyorRollers.conveyorRoller2
    subsystemPoseArray[11] = ConveyorRollers.conveyorRoller3
    subsystemPoseArray[12] = ConveyorRollers.dispatchRoller
    subsystemPoseArray[13] = FourBar.rollerLink
    subsystemPoseArray[14] = FourBar.upperLinkage
    subsystemPoseArray
}

@LoggedOutput(key = "Visualization/zeroedPoses", level = LogLevel.COMP)
val zeroedPoses = Array(17) { Pose3d() }

@LoggedOutput(key = "Visualization/zeroedPose", level = LogLevel.COMP)
val zeroedPose = Pose3d()