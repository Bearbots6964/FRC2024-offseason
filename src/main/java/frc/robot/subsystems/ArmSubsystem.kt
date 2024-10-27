package frc.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem

class ArmSubsystem : Subsystem {
    var motor: TalonSRX
    var pid: PIDController
    var config = TalonSRXConfiguration()
    var ff: ArmFeedforward
    var armCurrentDraw: NetworkTableEntry
    var armCurrentOutput: NetworkTableEntry

    init {
        // Initialize the arm subsystem
        motor = TalonSRX(31)
        motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder)
        // configure pid
        pid = PIDController(0.0, 0.0, 0.0)
        ff = ArmFeedforward(0.0, 0.0, 0.0)

//        config.slot0.kP = pid.p
//        config.slot0.kI = pid.i
//        config.slot0.kD = pid.d
//        config.slot0.kF = ff.
//
//        SmartDashboard.putData(pid)

        SmartDashboard.putNumber("Arm Current Draw", 0.0)
        armCurrentDraw = SmartDashboard.getEntry("Arm Current Draw")
        SmartDashboard.putNumber("Arm Current Output", 0.0)
        armCurrentOutput = SmartDashboard.getEntry("Arm Current Output")
    }

    // quadrature encoder
    override fun periodic() {
        // This method will be called once per scheduler run
        // check if pid controller changed
//        if ((kp != pid.p) || (ki != pid.i) || (kd != pid.d)) {
//            kp = pid.p
//            ki = pid.i
//            kd = pid.d
//        }

        armCurrentDraw.setNumber(motor.supplyCurrent)
        armCurrentOutput.setNumber(motor.statorCurrent)
    }

    fun extend(i: Double) {
        motor.set(ControlMode.PercentOutput, -0.4 * i)
    }

    fun retract(i: Double) {
        motor.set(ControlMode.PercentOutput, 0.4 * i)
    } // i love programming
    fun stop() {
        motor.set(ControlMode.PercentOutput, 0.0)
    }

    fun getCommand(joystickTriggerValue: () -> Double): Command {
        return run {
            if (joystickTriggerValue.invoke() > 0.1) {
                extend(joystickTriggerValue.invoke())
            } else if (joystickTriggerValue.invoke() < -0.1) {
                retract(
                    -joystickTriggerValue.invoke(),
                )
            }
        }
    }

    fun getSupplyCurrent(): Double = motor.supplyCurrent
    fun getOutputCurrent(): Double = motor.statorCurrent
}
