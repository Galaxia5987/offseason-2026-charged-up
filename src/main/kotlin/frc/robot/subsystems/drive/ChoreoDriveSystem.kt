package frc.robot.subsystems.drive
import choreo.trajectory.SwerveSample
import frc.robot.lib.getPose2d
import org.wpilib.math.controller.PIDController
import org.wpilib.math.geometry.Pose2d
import org.wpilib.math.kinematics.ChassisVelocities


val AutoXController = PIDController(10.0, 0.0, 0.0) // TODO config PID controllers

val AutoYController = PIDController(10.0, 0.0, 0.0)

val AutoHeadingController = PIDController(10.0, 0.0, 0.0)

 fun followTrajectory (sample : SwerveSample) {
  val pose: Pose2d = getPose2d()
  val speeds=
   ChassisVelocities(
      sample.vx + AutoXController.calculate(pose.getX(), sample.x),
      sample.vy + AutoYController.calculate(pose.getY(), sample.y),
      sample.omega + AutoHeadingController.calculate(
       pose.rotation.radians, sample.heading)
     )

 }
