package frc.robot.lib.align

import frc.robot.drive
import frc.robot.lib.TuneablePIDController
import frc.robot.lib.autopilot.APConstraints
import frc.robot.lib.autopilot.APProfile
import frc.robot.lib.autopilot.APTarget
import frc.robot.lib.autopilot.Autopilot
import frc.robot.lib.commands.UnnamedCommand
import frc.robot.lib.commands.command
import frc.robot.lib.extensions.cm
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.mps
import frc.robot.lib.extensions.rad_ps
import kotlin.math.PI
import org.wpilib.math.geometry.Pose2d
import org.wpilib.math.geometry.Rotation2d
import org.wpilib.math.kinematics.ChassisVelocities
import org.wpilib.units.measure.AngularVelocity
import org.wpilib.units.measure.Distance
import org.wpilib.units.measure.LinearVelocity

private val kConstraints = APConstraints().withAcceleration(5.0).withJerk(2.0)

private val kProfile =
    APProfile(kConstraints)
        .withErrorXY(2.cm)
        .withErrorTheta(0.5.deg)
        .withBeelineRadius(8.cm)

private val autopilot = Autopilot(kProfile)

private val anglePIDController =
    TuneablePIDController("alignAnglePIDController", 0.0, 0.0, 0.0)

private fun Autopilot.APResult.toChassisVelocities(
    omegaResult: AngularVelocity
) =
    ChassisVelocities(
        vx,
        vy,
        omegaResult,
    )

private inline fun <T, V> T.applyIfNotNull(value: V?, block: T.(V) -> T): T =
    if (value != null) block(value) else this

fun runToPose(
    targetSupplier: () -> Pose2d,
    entryAngle: Rotation2d? = null,
    endVelocity: LinearVelocity? = null,
    rotationRadius: Distance? = null,
): UnnamedCommand = command {
    val apTarget = {
        APTarget(targetSupplier())
            .applyIfNotNull(entryAngle) { withEntryAngle(it) }
            .applyIfNotNull(endVelocity) { withVelocity(it[mps]) }
            .applyIfNotNull(rotationRadius) { withRotationRadius(it) }
    }
    anglePIDController
        .update() // Update gains from network at the start of execution
    anglePIDController.enableContinuousInput(-PI, PI)

    while (true) {
        val target = apTarget()
        val result =
            autopilot.calculate(drive.pose, drive.chassisSpeeds, target)
        val omegaResult =
            anglePIDController.calculate(
                drive.pose.rotation.radians,
                result.targetAngle.radians,
            )

        drive.runVelocity(
            result
                .toChassisVelocities(omegaResult.rad_ps)
                .toRobotRelative(drive.pose.rotation)
        )
        if (autopilot.atTarget(drive.pose, target)) break
        yield()
    }
    drive.stop()
}

fun runToPose(
    target: Pose2d,
    entryAngle: Rotation2d? = null,
    endVelocity: LinearVelocity? = null,
    rotationRadius: Distance? = null,
): UnnamedCommand =
    runToPose({ target }, entryAngle, endVelocity, rotationRadius)
