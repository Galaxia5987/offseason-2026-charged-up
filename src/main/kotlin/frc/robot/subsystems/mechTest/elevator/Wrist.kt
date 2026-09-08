package frc.robot.subsystems.mechTest.elevator

import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.m
import frc.robot.lib.mechanism2d.MechanismBuilder
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.wpilib.command3.Mechanism

object Wrist : Mechanism() {

    private val motor = UniversalTalonFX(
        0
    )

    object Visualizer : MechanismBuilder() {

        val wristSection = section("wristSection", { 1.m }, motor.inputs::position)

        override val mechanism = root {
            wristSection.attach()
        }
    }
}