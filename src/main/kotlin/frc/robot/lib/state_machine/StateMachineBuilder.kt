package frc.robot.lib.state_machine

import java.util.function.BooleanSupplier
import org.littletonrobotics.junction.Logger
import org.wpilib.command3.Command
import org.wpilib.command3.Command.noRequirements
import org.wpilib.command3.Coroutine
import org.wpilib.command3.Scheduler
import org.wpilib.command3.StateMachine
import org.wpilib.command3.Trigger

class StateMachineBuilder<E : Enum<E>>(
    val name: String,
    val log: Boolean = true,
) {
    val stateMachine = StateMachine(name)

    private val stateMap = mutableMapOf<E, StateMachine.State>()

    private var stateChangeCallback: (newState: E) -> Unit = {}

    operator fun E.invoke(command: Command): E {
        require(!stateMap.containsKey(this))

        val state = stateMachine.addState(command)
        if (log) {
            val logPath =
                "States/${this@StateMachineBuilder::class.simpleName}/state"
            state.onEnter {
                Logger.recordOutput(logPath, name)
                stateChangeCallback(this)
            }
        }
        stateMap[this] = state
        return this
    }

    fun onStateChange(callback: (newState: E) -> Unit) {
        stateChangeCallback = callback
    }

    inline operator fun E.invoke(crossinline block: Coroutine.() -> Unit): E {
        this(noRequirements { coroutine -> coroutine.block() }.named(name))
        return this
    }

    fun E.initial() {
        stateMachine.setInitialState(getState(this))
    }

    private fun getState(enumVal: E): StateMachine.State {
        return stateMap[enumVal]
            ?: error(
                "State $enumVal was used in a transition but never defined."
            )
    }

    enum class TransitionType {
        ON,
        COMPLETE_AND,
    }

    inner class TransitionCondition(
        val source: E,
        val condition: Trigger,
        val transitionType: TransitionType,
    )

    inner class MultiTransitionCondition(
        val sources: List<E>,
        val condition: Trigger,
    )

    inner class CompleteTransitionWrapper(val source: E)

    infix fun E.on(trigger: Trigger): TransitionCondition =
        TransitionCondition(this, trigger, TransitionType.ON)

    infix fun E.on(condition: () -> Boolean): TransitionCondition =
        on(Trigger(condition))

    infix fun E.completeAnd(trigger: Trigger): TransitionCondition =
        TransitionCondition(this, trigger, TransitionType.COMPLETE_AND)

    infix fun E.completeAnd(condition: () -> Boolean): TransitionCondition =
        completeAnd(Trigger(condition))

    infix fun TransitionCondition.switchTo(target: E) {
        getState(this.source).switchTo(getState(target)).apply {
            when (transitionType) {
                TransitionType.ON -> `when`(condition)
                TransitionType.COMPLETE_AND -> whenCompleteAnd(condition)
            }
        }
    }

    inline fun <reified E : Enum<E>> allOf(): List<E> = enumValues<E>().toList()

    infix fun List<E>.on(trigger: Trigger): MultiTransitionCondition =
        MultiTransitionCondition(this, trigger)

    infix fun List<E>.on(condition: () -> Boolean): MultiTransitionCondition =
        on(Trigger(condition))

    infix fun MultiTransitionCondition.switchTo(target: E) {
        val sourceStates = this.sources.map { getState(it) }.toTypedArray()
        stateMachine
            .switchFromAny(*sourceStates)
            .to(getState(target))
            .`when`(this.condition)
    }

    val E.onComplete: CompleteTransitionWrapper
        get() = CompleteTransitionWrapper(this)

    infix fun CompleteTransitionWrapper.switchTo(target: E) {
        getState(this.source).switchTo(getState(target)).whenComplete()
    }

    infix fun Trigger.and(other: BooleanSupplier): Trigger = this.and(other)

    infix fun Trigger.or(other: BooleanSupplier): Trigger = this.and(other)
}

inline fun <reified E : Enum<E>> buildStateMachine(
    name: String,
    init: StateMachineBuilder<E>.() -> Unit,
): StateMachine {
    return StateMachineBuilder<E>(name).apply(init).stateMachine
}

fun StateMachine.register() {
    Scheduler.getDefault().schedule(this)
}
