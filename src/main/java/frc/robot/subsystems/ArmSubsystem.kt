package frc.robot.subsystems

import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem

object ArmSubsystem : Subsystem {
    var motor: TalonFX
    override fun periodic() {
        // This method will be called once per scheduler run
    }

    init {
        // Initialize the arm subsystem
        motor = TalonFX(31)
    }

    fun extend(i: Double) {
        motor.set(-0.4 * i)
    }

    fun retract(i: Double) {
        motor.set(0.4 * i)
    } // i love programming

    fun getCommand(joystickTriggerValue: () -> Double): Command {
        return run {
            if (joystickTriggerValue.invoke() > 0.1) extend(joystickTriggerValue.invoke()) else if (joystickTriggerValue.invoke() < -0.1) retract(
                -joystickTriggerValue.invoke()
            )
        }

    }
}