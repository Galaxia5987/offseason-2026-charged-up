package frc.robot.subsystems.leds

import com.ctre.phoenix6.CANBus.systemcore
import com.ctre.phoenix6.controls.LarsonAnimation
import com.ctre.phoenix6.controls.RainbowAnimation
import com.ctre.phoenix6.controls.SolidColor
import com.ctre.phoenix6.controls.StrobeAnimation
import com.ctre.phoenix6.hardware.CANdle
import com.ctre.phoenix6.signals.RGBWColor
import frc.robot.field.CubeColors
import frc.robot.lib.commands.invoke
import frc.robot.states.State
import frc.robot.subsystems.sensors.Sensors
import org.wpilib.command3.Command
import org.wpilib.command3.Mechanism

object Leds : Mechanism() {

    private val candle = CANdle(39, systemcore(0))
    private val solidColorRequest = SolidColor(full.first, full.last)
    private val flickerRequest = StrobeAnimation(full.first, full.last)
    private val rainbowRequest = RainbowAnimation(full.first, full.last)
    private val chaseRequest =
        LarsonAnimation(full.first, full.last).withSize(5)

    init {
        defaultCommand = cubesPresent()
    }

    val intaking = State.trigger(State.INTAKING).whileTrue(intaking())

    fun intaking(): Command =
        this {
                while (true) {
                    candle.setControl(flickerRequest.withColor(BLUE))
                    yield()
                }
            }
            .named("Leds/intaking")

    fun cubesPresent(): Command =
        runRepeatedly {
                setColor(Sensors.bodySensor.color, segmentBody)
                setColor(Sensors.dispatchSensor.color, segmentDispatch)
                setColor(Sensors.gripSensor.color, segmentGrip)
            }
            .named("Leds/intaking")

    fun setColor(color: CubeColors, index: IntRange) {
        candle.setControl(
            solidColorRequest.withColor(color.toRGBWColor()).withIndex(index)
        )
    }
}

fun CubeColors.toRGBWColor() = RGBWColor(this.color)

fun SolidColor.withIndex(range: IntRange): SolidColor =
    withLEDStartIndex(range.first).withLEDEndIndex(range.last)
