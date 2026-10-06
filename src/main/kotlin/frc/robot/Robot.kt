// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.pathplanner.lib.util.FlippingUtil
import frc.robot.field.EmptyArena
import frc.robot.lib.BetterPoseEstimator
import frc.robot.lib.extensions.CacheManager
import frc.robot.lib.extensions.enableAutoLogOutputFor
import frc.robot.lib.extensions.toPose3d
import frc.robot.lib.logged_output.LoggedOutputManager
import org.littletonrobotics.junction.*
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter
import org.wpilib.command3.Command
import org.wpilib.command3.Scheduler
import org.wpilib.driverstation.internal.DriverStationBackend
import org.wpilib.hardware.power.PowerDistribution
import org.wpilib.math.linalg.VecBuilder
import org.wpilib.system.Timer

val arena = EmptyArena()

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
class Robot : LoggedRobot() {
    private lateinit var autonomousCommand: Command

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    init {
        FlippingUtil.symmetryType = FlippingUtil.FieldSymmetry.kMirrored
        arrayOf(vision, drive).forEach { AutoLogOutputManager.addObject(it) }

        // Initialize logger
        listOf(
                "Project name" to BuildConstants.MAVEN_NAME,
                "Build date" to BuildConstants.BUILD_DATE,
                "Last commit hash" to BuildConstants.GIT_SHA,
                "Last commit timestamp" to BuildConstants.GIT_DATE,
                "Branch" to BuildConstants.GIT_BRANCH,
            )
            .forEach { (key, value) -> Logger.recordMetadata(key, value) }
        @Suppress("KotlinConstantConditions")
        Logger.recordMetadata(
            "Diff status",
            when (BuildConstants.DIRTY) {
                0 -> "All changes committed"
                1 -> "Uncommitted changes"
                else -> "Unknown"
            },
        )

        when (CURRENT_MODE) {
            frc.robot.lib.Mode.REAL -> {
                LoggedPowerDistribution.getInstance(
                    0,
                    1,
                    PowerDistribution.ModuleType.REV,
                )
                Logger.addDataReceiver(WPILOGWriter())
                Logger.addDataReceiver(NT4Publisher())
            }
            frc.robot.lib.Mode.SIM -> Logger.addDataReceiver(NT4Publisher())
            frc.robot.lib.Mode.REPLAY -> {
                setUseTiming(false)
                val logPath = LogFileUtil.findReplayLog()
                Logger.setReplaySource(WPILOGReader(logPath))
                Logger.addDataReceiver(
                    WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_replay"))
                )
            }
        }
        Logger.start()
        RobotContainer // Initialize robot container.

        enableAutoLogOutputFor(this)

        LoggedOutputManager

        DriverStationBackend.silenceJoystickConnectionWarning(true)

        // TODO: Update when PathPlanner works with CommandsV3
        //        Scheduler.getDefault()
        //            .schedule(
        //                FollowPathCommand.warmupCommand(),
        //                PathfindingCommand.warmupCommand()
        //            )
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this
     * for items like diagnostics that you want ran during disabled, autonomous,
     * teleoperated and test.
     *
     * This runs after the mode specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
     */
    override fun robotPeriodic() {
        CacheManager.invalidateAll()
        Scheduler.getDefault().run()
        Logger.recordOutput("Scheduler", Scheduler.getDefault())
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable
     * chooser code works with the Java SmartDashboard. If you prefer the
     * LabVIEW Dashboard, remove all of the chooser code and uncomment the
     * getString line to get the auto name from the text box below the Gyro
     *
     * You can add additional auto modes by adding additional comparisons to the
     * switch structure below with additional strings. If using the
     * SendableChooser make sure to add them to the chooser code above as well.
     */
    override fun autonomousInit() {
        // Make sure command is compiled beforehand, otherwise there will be a
        // delay.
        autonomousCommand = RobotContainer.getAutonomousCommand()

        // Schedule the autonomous command
        Scheduler.getDefault().schedule(autonomousCommand)
    }

    /** This function is called periodically during autonomous. */
    override fun autonomousPeriodic() {}

    /** This function is called once when teleop is enabled. */
    override fun teleopInit() {
        if (::autonomousCommand.isInitialized) {
            autonomousCommand.raceWith(
                Command.noRequirements {}.named("CancelAutonomousCommand")
            ) // TODO: Make this less shitty
        }
    }

    override fun simulationInit() {
        resetSimulationField()
    }

    override fun simulationPeriodic() {
        val pose = getMapleSimPose()!!.toPose3d()
        val timestamp = Timer.getTimestamp()
        val stdDevs = VecBuilder.fill(0.01, 0.01, 0.01)
        val observation =
            BetterPoseEstimator.VisionObservation(
                pose,
                timestamp,
                stdDevs.get(0),
                stdDevs.get(1),
                stdDevs.get(1),
            )

        BetterPoseEstimator.getInstance().addVisionObservation(observation)
        arena.simulationPeriodic()
    }

    /** This function is called periodically during operator control. */
    override fun teleopPeriodic() {}

    /** This function is called once when the robot is disabled. */
    override fun disabledInit() {}

    /** This function is called periodically when disabled. */
    override fun disabledPeriodic() {}
}
