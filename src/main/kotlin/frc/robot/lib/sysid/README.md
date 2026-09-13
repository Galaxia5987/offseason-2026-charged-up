# SysIdCommand Factory

`SysIdCommand` is a Kotlin-based helper for characterizing FRC subsystems using WPILib's SysId framework. It simplifies the creation of dynamic and quasistatic tests for any subsystem that implements the `SysIdable` interface and extends `SubsystemBase`.

---
## Usage

### Implement `SysIdable`

To use `SysIdCommand`, your subsystem must implement the `SysIdable` interface, which requires a method to apply voltage to the mechanism.

```kotlin
object SysIdTest : Mechanism(), SysIdable {

    private val motor = UniversalTalonFX(0)

    override fun setVoltage(voltage: Voltage) = makeSysIdVoltageSupplier(motor, voltage)

    override fun configureSysId(): SysIdMechanismConfig = buildSysIdConfig {
        symmetric {
            rampRate = 1.0.volts.per(sec)
            stepVoltage = 6.0.volts
            timeout = 5.0.sec
        }
    }

}
````

### Create and Configure a `SysIdCommand`

Use the `sysId()` extension function to generate a `SysIdCommand` and configure it with forward and backward routines.

```kotlin
val sysIdCommand = wrist.sysid()
```