package frc.robot.subsystems.mechTest.elevator

import frc.robot.lib.extensions.deg
import frc.robot.lib.mechanism2d.MechanismBuilder
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.wpilib.command3.Mechanism

object Elevator : Mechanism() {

    private val motor = UniversalTalonFX(
        0
    )

    object Visualizer : MechanismBuilder() {
        val elevatorSection = section("elevatorSection", motor.inputs::distance, {90.deg}) {
            Wrist.Visualizer.wristSection.attach()
        }

        override val mechanism = root {
            elevatorSection.attach()
        }
    }

    fun periodic() {
        Visualizer.update()
    }
}