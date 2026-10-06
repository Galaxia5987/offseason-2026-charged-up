// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.state_machine

import org.littletonrobotics.junction.Logger
import org.wpilib.command3.Command
import org.wpilib.command3.Command.noRequirements
import org.wpilib.command3.Coroutine
import org.wpilib.command3.Mechanism
import org.wpilib.command3.Scheduler
import org.wpilib.command3.Trigger
import java.util.function.BooleanSupplier
import kotlin.reflect.KClass

/**
 * A declarative state machine that can be used to implement complex command routines.
 * This combines the execution logic of the WPILib StateMachine with a native Kotlin DSL.
 */
class StateMachine<E : Enum<E>>(
    private val name: String,
    private val log: Boolean = true,
) : Command {

    private val stateMap = mutableMapOf<E, State>()
    private var initialState: State? = null
    private var stateChangeCallback: (newState: E) -> Unit = {}

    override fun name(): String = name

    override fun requirements(): Set<Mechanism> = emptySet()

    // --------------------------------------------------------
    // State Machine Execution Logic
    // --------------------------------------------------------

    override fun run(coroutine: Coroutine) {
        var currentState = initialState
            ?: error("$name does not have an initial state. Use .initial() to provide one.")

        outer_loop@ while (true) {
            val currentCommand = currentState.command
            coroutine.fork(currentCommand)
            currentState.runEnterCallbacks()
            var didYield = false

            while (true) {
                for (transition in currentState.transitions) {
                    if (transition.shouldTransition()) {
                        currentState.runExitCallbacks()
                        coroutine.scheduler().cancel(currentCommand)

                        val nextState = transition.nextState()
                        currentState = nextState ?: initialState!!
                        continue@outer_loop
                    }
                }

                if (!coroutine.scheduler().isRunning(currentCommand)) {
                    val next = currentState.nextState()
                    if (next != null) {
                        currentState.runExitCallbacks()
                        currentState = next
                        if (!didYield) coroutine.yield()
                        continue@outer_loop
                    }
                }

                coroutine.yield()
                didYield = true
            }
        }
    }

    // --------------------------------------------------------
    // Internal Components
    // --------------------------------------------------------

    inner class State(val enumVal: E, val command: Command) {
        val completions = mutableListOf<Completion>()
        var defaultNextState: (() -> State?)? = null
        val transitions = mutableListOf<Transition>()
        val enterCallbacks = mutableListOf<Runnable>()
        val exitCallbacks = mutableListOf<Runnable>()

        fun addTransition(transition: Transition) {
            transitions.add(transition)
        }

        fun addCompletion(condition: BooleanSupplier, next: () -> State?) {
            completions.removeIf { it.condition === condition }
            completions.add(Completion(next, condition))
        }

        fun nextState(): State? {
            for (completion in completions) {
                if (completion.shouldTransition()) {
                    return completion.nextState()
                }
            }
            return defaultNextState?.invoke()
        }

        fun runEnterCallbacks() = enterCallbacks.forEach { it.run() }
        fun runExitCallbacks() = exitCallbacks.forEach { it.run() }

        fun onEnter(callback: Runnable) {
            enterCallbacks.add(callback)
        }

        fun onExit(callback: Runnable) {
            exitCallbacks.add(callback)
        }
    }

    inner class Completion(
        private val nextSupplier: () -> State?,
        val condition: BooleanSupplier
    ) {
        fun shouldTransition(): Boolean = condition.asBoolean
        fun nextState(): State? = nextSupplier()
    }

    inner class Transition(
        private val nextSupplier: () -> State?,
        private val condition: BooleanSupplier
    ) {

        fun shouldTransition(): Boolean = condition.asBoolean

        fun nextState(): State? = nextSupplier()
    }

    // --------------------------------------------------------
    // Kotlin DSL
    // --------------------------------------------------------

    fun onStateChange(callback: (newState: E) -> Unit) {
        stateChangeCallback = callback
    }

    inline fun <reified S : Enum<S>> allOf(): List<S> = enumValues<S>().toList()

    operator fun E.invoke(command: Command): E {
        require(!stateMap.containsKey(this)) { "State $this is already defined." }

        val state = State(this, command)
        if (log) {
            val logPath = "States/${this::class.simpleName}/state"
            state.onEnter {
                Logger.recordOutput(logPath, name)
            }
        }
        state.onEnter { stateChangeCallback(this) }
        stateMap[this] = state
        return this
    }

    operator fun E.invoke(block: Coroutine.() -> Unit): E {
        this(noRequirements { coroutine -> coroutine.block() }.named(name))
        return this
    }

    fun E.initial() {
        initialState = getState(this)
    }

    private fun getState(enumVal: E): State {
        return stateMap[enumVal]
            ?: error("State $enumVal was used in a transition but never defined.")
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
        val sourceState = getState(this.source)
        val targetSupplier = { getState(target) }

        when (transitionType) {
            TransitionType.ON -> {
                sourceState.addTransition(Transition(targetSupplier, condition))
            }
            TransitionType.COMPLETE_AND -> {
                sourceState.addCompletion(condition, targetSupplier)
            }
        }
    }

    infix fun List<E>.on(trigger: Trigger): MultiTransitionCondition =
        MultiTransitionCondition(this, trigger)

    infix fun List<E>.on(condition: () -> Boolean): MultiTransitionCondition =
        on(Trigger(condition))

    infix fun MultiTransitionCondition.switchTo(target: E) {
        val targetSupplier = { getState(target) }
        this.sources.forEach { sourceEnum ->
            getState(sourceEnum).addTransition(Transition(targetSupplier, condition))
        }
    }


    val E.onComplete: CompleteTransitionWrapper
        get() = CompleteTransitionWrapper(this)

    infix fun CompleteTransitionWrapper.switchTo(target: E) {
        getState(this.source).defaultNextState = { getState(target) }
    }

}

// --------------------------------------------------------
// Utilities & Extensions
// --------------------------------------------------------

inline fun <reified E : Enum<E>> allOf(): List<E> = enumValues<E>().toList()

fun <E : Enum<E>> buildStateMachine(
    name: String,
    init: StateMachine<E>.() -> Unit,
): StateMachine<E> {
    return StateMachine<E>(name).apply(init)
}

fun Command.register() {
    Scheduler.getDefault().schedule(this)
}

@RequiresOptIn
annotation class Unsafe

abstract class StateMachineCompanion<T : Enum<T>>(stateClass: KClass<T>) {
    var state: T = stateClass.java.enumConstants.first()
        private set

    protected abstract val states: StateMachine<T>.() -> Unit

    protected fun makeStates(
        init: StateMachine<T>.() -> Unit
    ): StateMachine<T>.() -> Unit = init

    fun trigger(state: T) = Trigger { state == this.state }

    @Unsafe
    fun set(state: T) {
        this.state = state
    }

    private val stateMachine by lazy {
        buildStateMachine<T>(stateClass.simpleName!!) {
            states()
            onStateChange { state = it }
        }
    }

    fun register() = stateMachine.register()
}
