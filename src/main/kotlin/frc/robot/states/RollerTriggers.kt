package frc.robot.states
import org.wpilib.command3.Trigger
import org.wpilib.command3.Command
import frc.robot.subsystems.roller.IntakeRoller
import frc.robot.subsystems.roller.ConveyorRoller
import frc.robot.states.StateMachine
import frc.robot.subsystems.roller.DispatchRoller
import frc.robot.subsystems.roller.GripRoller
import frc.robot.subsystems.sensors.Sensors.bodySensor
import frc.robot.subsystems.sensors.Sensors.GripSensor
import frc.robot.subsystems.sensors.Sensors.DispatchSensor


val intakeRollerStop = Trigger{
    bodySensor.isPresent && DispatchSensor.isPresent
}.whileTrue(IntakeRoller.stop())


val conveyorRollerStop = Trigger{
    StateMachine.IDLE
}.whileTrue(ConveyorRoller.stop())


val dispatchRollerStop = Trigger{
    (StateMachine.SCORINGHIGH && GripSensor.isPresent) || (StateMachine.SCORINGLOW && StateMachine.cubeLeavesBody)
}.whileTrue(DispatchRoller.stop())


val gripRollerStop = Trigger{
    (GripSensor.isPresent)
}.whileTrue(GripRoller.stop())