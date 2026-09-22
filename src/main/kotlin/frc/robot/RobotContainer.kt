package frc.robot

import frc.robot.lib.BasicAlerts
import frc.robot.lib.MechanismRegistry
import frc.robot.lib.Mode
import frc.robot.lib.commands.emptyCommand
import frc.robot.lib.commands.initializeAllMechanisms
import frc.robot.lib.extensions.enableAutoLogOutputFor
import frc.robot.lib.sysid.SysIdable
import frc.robot.lib.sysid.sysId
import frc.robot.lib.toReleaseTrigger
import frc.robot.lib.unified_controller.PS5Gamepad
import frc.robot.subsystems.drive.DriveCommands
import org.ironmaple.simulation.SimulatedArena
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import org.wpilib.command3.Command
import org.wpilib.smartdashboard.SendableChooser

object RobotContainer {
    private val driverController = PS5Gamepad(0)
    private val autoChooser: LoggedDashboardChooser<Command>

    object Buttons {
        var scoring = driverController.leftBumper().toReleaseTrigger()
        var intake = driverController.rightBumper()
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
    }

    private fun configureDefaultCommands() {
        drive.defaultCommand =
            DriveCommands.joystickDrive(
                { -driverController.leftY },
                { -driverController.leftX },
                { -driverController.rightX },
            )
    }

    private fun configureButtonBindings() {
        driverController.create().onTrue(DriveCommands.resetGyro())
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

        autoChooser.addOption(
            "swerveFFCharacterization",
            DriveCommands.feedforwardCharacterization(),
        )

        // Register all mechanisms that implement SysIdable
        MechanismRegistry.allMechanisms.forEach { mechanismClass ->
            val mechanism = mechanismClass.objectInstance
            if (mechanism is SysIdable) {
                autoChooser.addOption(
                    "${mechanism.name} SysId",
                    mechanism.sysId(),
                )
            }
        }
    }
}
