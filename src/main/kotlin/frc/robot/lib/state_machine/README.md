# State Machine Builder DSL

Usage Example:

```kotlin
enum class ShooterState {
    IDLE,
    SPIN_UP,
    SHOOT,
    COOLDOWN;

    companion object : StateMachineCompanion<State>(State::class){
        override val states = makeStates {
            IDLE {
                shooter.stop()
            }.initial()

            SPIN_UP(shooter.runSpinUpCommand())

            SHOOT {
                shooter.feedNote()
            }

            COOLDOWN {
                shooter.idleFlywheels()

            }

            IDLE on shootTrigger switchTo SPIN_UP

            SPIN_UP on isAtTargetSpeed switchTo SHOOT

            SHOOT.onComplete switchTo COOLDOWN

            COOLDOWN.onComplete switchTo IDLE

            allOf<ShooterState>() on { !shootTrigger.asBoolean } switchTo IDLE
        }
    }
}
```