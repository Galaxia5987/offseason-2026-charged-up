package frc.robot.lib.unified_controller

import org.wpilib.command3.Trigger
import org.wpilib.command3.button.CommandGamepad

class PS5Gamepad(port: Int) : CommandGamepad(port) {
    fun cross(): Trigger = super.eastFace()

    fun circle(): Trigger = super.westFace()

    fun triangle(): Trigger = super.northFace()

    fun square(): Trigger = super.southFace()

    fun options(): Trigger = super.button(10)

    fun create(): Trigger = super.button(9)
}
