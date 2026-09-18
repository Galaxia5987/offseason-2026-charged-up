package frc.robot.lib

import frc.robot.lib.commands.command
import frc.robot.lib.extensions.not
import org.wpilib.command3.Command
import org.wpilib.command3.Trigger

class ReleaseTrigger (buttonTrigger : Trigger) {

    enum class ToggleState{
        IDLE,
        PRESSED,
        RELEASED
    }
    private val ToggleState.trigger
        get() = Trigger {state == this@trigger}

    private fun ToggleState.set(): Command = command{
        state = this@set
    }.named("ReleaseTrigger/$name/set")

    private var state = ToggleState.IDLE

    private val onPressed = buttonTrigger.and(ToggleState.IDLE.trigger).onTrue(ToggleState.PRESSED.set())
    private val onReleased = (!buttonTrigger).and(ToggleState.PRESSED.trigger).onTrue(ToggleState.RELEASED.set())

    private fun get() : Boolean {
        return (state == ToggleState.RELEASED).also {
            state = ToggleState.IDLE
        }
    }

    val trigger = Trigger {get()}
}
fun Trigger.toReleaseTrigger() = ReleaseTrigger(this)

