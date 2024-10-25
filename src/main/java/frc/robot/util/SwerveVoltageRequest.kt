package frc.robot.util

import com.ctre.phoenix6.StatusCode
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SwerveControlRequestParameters

class SwerveVoltageRequest(private var driveType: Boolean) : SwerveRequest {
    private val motionMagicControl: MotionMagicVoltage =
        MotionMagicVoltage(
            /* Position = */ 0.0,
            /* EnableFOC = */ false,
            /* FeedForward = */ 0.0,
            /* Slot = */ 0,
            /* OverrideBrakeDurNeutral = */ false,
            /* LimitForwardMotion = */ false,
            /* LimitReverseMotion = */ false,
        )
    private val voltageOutControl: VoltageOut = VoltageOut(0.0)
    private var targetVoltage: Double = 0.0
    
    override fun apply(
        parameters: SwerveControlRequestParameters,
        vararg modulesToApply: SwerveModule,
    ): StatusCode {
        for (module in modulesToApply) {
            if (driveType) {
                // Command steer motor to zero
                module.steerMotor.setControl(motionMagicControl)
                
                // Command drive motor to voltage
                module.driveMotor.setControl(voltageOutControl.withOutput(targetVoltage))
            } else {
                // Command steer motor to voltage
                module.steerMotor.setControl(voltageOutControl.withOutput(targetVoltage))
                
                // Command drive motor to zero
                module.driveMotor.setControl(motionMagicControl)
            }
        }
        
        return StatusCode.OK
    }
    
    /**
     * @param targetVoltage Voltage for all modules to target
     * @return
     */
    fun withVoltage(targetVoltage: Double): SwerveVoltageRequest {
        this.targetVoltage = targetVoltage
        
        return this
    }
}
