# CommandEnum Subsystem Example

## Overview

The `Wrist` subsystem demonstrates how to use an annotated `WristAngles` enum to create clean, type-safe command methods.

## Enum Definition

Define your angles using the `@CommandEnum` annotation:
```kotlin
@CommandEnum
enum class WristAngles(val angle: Angle) {
    CLOSED(0.rot),
    OPEN(15.5.rot),
    DEFAULT(2.4.rot)
}
```

## Subsystem Implementation

Extend the generated command factory and define the angle-setting behavior:
```kotlin
object Wrist : WristAnglesCommandFactory() {
    ..
    
    protected override fun setTarget(value: WristAngles): Command = runOnce({
        setpoint = value.angle
        motor.setControl(positionRequest.withPosition(value.angle))
    })
    
    ..
}
```

## Command binding

Bind the generated command methods to controller buttons.
```kotlin
object RobotContainer {
    ..
    fun configureButtonBindings() {
        controller.a().onTrue(Wrist.closed())
    }
    ..
}
```

## Priorities

Command Enums may also include a command priority for each enum entry, this is done like the following:
```kotlin
@CommandEnum
enum class WristAngles(val angle: Angle, val priority: Priority) {
    CLOSED(0.rot, Priority(Command.LOWEST_PRIORITY)),
    OPEN(15.5.rot, Priority(Command.HIGHEST_PRIORITY)),
    DEFAULT(2.4.rot, Priority(Command.DEFAULT_PRIORITY))
}
```

> Note that the processor looks for the `Priority` type in the primary constructor, and thus the property can be called or placed as wished, as long as it's type is `Priority`
