package frc.robot.states

import frc.robot.lib.commands.command
import frc.robot.lib.commands.unaryPlus
import frc.robot.setpoint_manager.SetpointManager
import frc.robot.subsystems.hood.Hood
import frc.robot.subsystems.preShooter.PreShooter
import frc.robot.subsystems.shooter.flywheel.Flywheel
import frc.robot.subsystems.spindexer.Spindexer
import frc.robot.subsystems.turret.Turret
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput
import org.wpilib.command3.Trigger

@LoggedOutput(LogLevel.COMP) var shouldShoot = true

@LoggedOutput(LogLevel.COMP)
val isReadyToShoot =
    Flywheel.atSetpoint.and(Turret.atSetpoint).and(Hood.atSetpoint)

private val convey =
    command {
            +PreShooter.convey()
            +Spindexer.convey()
            park()
        }
        .named("states/Shooting/Convey")

private val stopConveyor =
    command {
            +[
                PreShooter.stop(),
                Spindexer.stop(),
            ]
            park()
        }
        .named("states/Shooting/StopConveyor")

val shouldShootTrigger =
    Trigger { shouldShoot }
        .whileTrue(Flywheel.setVelocity { SetpointManager.flywheelSetpoint })

@LoggedOutput(LogLevel.COMP)
val shoot =
    shouldShootTrigger.and(isReadyToShoot).onTrue(convey).onFalse(stopConveyor)
