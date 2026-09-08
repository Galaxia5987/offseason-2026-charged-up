package frc.robot

import frc.robot.RobotContainer.Buttons.intakeButton
import frc.robot.RobotContainer.Buttons.outtakeButton
import frc.robot.lib.BasicAlerts
import frc.robot.lib.Mode
import frc.robot.lib.commands.command
import frc.robot.lib.commands.emptyCommand
import frc.robot.lib.commands.initializeAllMechanisms
import frc.robot.lib.extensions.enableAutoLogOutputFor
import frc.robot.lib.extensions.logTrigger
import frc.robot.lib.state_machine.register
import frc.robot.lib.unified_controller.PS5Gamepad
import frc.robot.setpoint_manager.SetpointManager
import frc.robot.states.IntakeState
import frc.robot.states.setShouldShoot
import frc.robot.subsystems.drive.DriveCommands
import frc.robot.subsystems.hood.Hood
import frc.robot.subsystems.turret.Turret
import org.ironmaple.simulation.SimulatedArena
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import org.wpilib.command3.Command
import org.wpilib.command3.Scheduler
import org.wpilib.command3.Trigger
import org.wpilib.smartdashboard.SendableChooser

object RobotContainer {
    private val driverController = PS5Gamepad(0)
    private val autoChooser: LoggedDashboardChooser<Command>

    object Buttons {
        var intakeButton = Trigger { false }
        var outtakeButton = Trigger { false }
    }

    init {
        drive // Ensure Drive is initialized
        autoChooser =
            LoggedDashboardChooser(
                "Auto Choices",
                SendableChooser(),
            )
        registerAutoCommands()
        configureButtonBindings()
        configureDefaultCommands()

        if (CURRENT_MODE == Mode.SIM) {
            SimulatedArena.getInstance()
                .addDriveTrainSimulation(driveSimulation)
            SimulatedArena.getInstance().resetFieldForAuto()
        }

        enableAutoLogOutputFor(this)
        initializeAllMechanisms()
        BasicAlerts
        SetpointManager
    }

    private fun configureDefaultCommands() {
        drive.defaultCommand =
            DriveCommands.joystickDrive(
                { -driverController.leftY },
                { -driverController.leftX },
                { -driverController.rightX },
            )

        Turret.defaultCommand = Turret.setAngle {
            SetpointManager.turretSetpoint
        }
        Hood.defaultCommand = Hood.setPosition { SetpointManager.hoodSetpoint }


        IntakeState.stateMachine.register()
    }

    private fun configureButtonBindings() {
        driverController.create().onTrue(DriveCommands.resetGyro())
        intakeButton = driverController.rightBumper()
        outtakeButton = driverController.leftBumper()
        driverController.circle().onFalse( setShouldShoot(true)).onTrue(setShouldShoot(false))
    }

    fun getAutonomousCommand(): Command = autoChooser.get()

    private fun registerAutoCommands() {
        autoChooser.addDefaultOption("Empty", emptyCommand())

        // SysIds
        autoChooser.addOption(
            "Drive Wheel Radius Characterization",
            DriveCommands.wheelRadiusCharacterization(),
        )
        autoChooser.addOption(
            "Drive Simple FF Characterization",
            DriveCommands.feedforwardCharacterization(),
        )

        // TODO: Uncomment when I figure out what happened to SysId
        //        autoChooser.addOption(
        //            "Drive SysId (Quasistatic Forward)",
        //            drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        //        )
        //        autoChooser.addOption(
        //            "Drive SysId (Quasistatic Reverse)",
        //            drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
        //        )
        //        autoChooser.addOption(
        //            "Drive SysId (Dynamic Forward)",
        //            drive.sysIdDynamic(SysIdRoutine.Direction.kForward)
        //        )
        //        autoChooser.addOption(
        //            "Drive SysId (Dynamic Reverse)",
        //            drive.sysIdDynamic(SysIdRoutine.Direction.kReverse)
        //        )

        autoChooser.addOption(
            "swerveFFCharacterization",
            DriveCommands.feedforwardCharacterization(),
        )
    }
}
