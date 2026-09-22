package frc.robot.subsystems.leds

import com.ctre.phoenix6.CANBus.systemcore
import com.ctre.phoenix6.controls.LarsonAnimation
import com.ctre.phoenix6.controls.RainbowAnimation
import com.ctre.phoenix6.controls.SolidColor
import com.ctre.phoenix6.controls.StrobeAnimation
import com.ctre.phoenix6.hardware.CANdle
import frc.robot.field.CubeColors
import frc.robot.lib.commands.invoke
import frc.robot.states.State
import frc.robot.subsystems.sensors.Sensors
import org.wpilib.command3.Command
import org.wpilib.command3.Mechanism

object Leds : Mechanism() {

    private val candle = CANdle(39, systemcore(0))
    private val solidColorRequest = SolidColor(full.first, full.second)
    private val flickerRequest = StrobeAnimation(full.first, full.second)
    private val rainbowRequest = RainbowAnimation(full.first, full.second)
    private val chaseRequest =
        LarsonAnimation(full.first, full.second).withSize(5)

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
        this {
                while (true) {
                    candle.setControl(
                        solidColorRequest
                            .withCubeColor(Sensors.BodySensor.color)
                            .withIndex(segment1)
                    )
                    candle.setControl(
                        solidColorRequest
                            .withCubeColor(Sensors.DispatchSensor.color)
                            .withIndex(segment2)
                    )
                    candle.setControl(
                        solidColorRequest
                            .withCubeColor(Sensors.GripSensor.color)
                            .withIndex(segment3)
                    )
                    yield()
                }
            }
            .named("Leds/intaking")
}

fun SolidColor.withCubeColor(color: CubeColors): SolidColor =
    withColor(
        when (color) {
            CubeColors.GREEN -> GREEN
            CubeColors.YELLOW -> YELLOW
            CubeColors.RED -> RED
            CubeColors.NONE -> BLACK
        }
    )

fun SolidColor.withIndex(pair: Pair<Int, Int>): SolidColor =
    withLEDStartIndex(pair.first).withLEDEndIndex(pair.second)
