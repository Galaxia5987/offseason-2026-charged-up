package frc.robot.lib.commands

import org.wpilib.command3.Command
import org.wpilib.command3.Coroutine
import org.wpilib.command3.Mechanism
import org.wpilib.command3.NeedsNameBuilderStage
import org.wpilib.command3.Trigger

/**
 * A typealias for a command that hasn't finished it's building. This command
 * cannot be scheduled and `.named` must be called.
 */
typealias UnnamedCommand = NeedsNameBuilderStage

fun emptyCommand(): Command =
    Command.noRequirements(Coroutine::park)
        .withPriority(Command.LOWEST_PRIORITY)
        .named("EmptyCommand")

context(coroutine: Coroutine)
operator fun Command.unaryPlus() = coroutine.await(this)

context(coroutine: Coroutine)
operator fun List<Command>.unaryPlus() =
    coroutine.awaitAll(*(this.toTypedArray()))

inline fun command(crossinline block: Coroutine.() -> Unit): UnnamedCommand {
    return Command.noRequirements { coroutine ->
        coroutine.block()
    }
}

inline operator fun Mechanism.invoke(
    crossinline block: Coroutine.() -> Unit
): UnnamedCommand {
    return this.run { coroutine ->
        coroutine.block()
    }
}

context(coroutine: Coroutine)
fun Trigger.waitUntil() {
    do {
        coroutine.yield()
    } while (!this.asBoolean)
}

fun Trigger.onChange(command: Command) {
    onFalse(command)
    onTrue(command)
}

fun Trigger.onChange(runnable: (state: Boolean) -> Unit) {
    onFalse(command { runnable(false) }.named("onChangeFalse"))
    onTrue(command { runnable(true) }.named("onChangeTrue"))
}
