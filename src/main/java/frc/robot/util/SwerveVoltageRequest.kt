package frc.robot.util

import com.ctre.phoenix6.StatusCode
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SwerveControlRequestParameters

class SwerveVoltageRequest : SwerveRequest {
    private val m_motionMagicControl: MotionMagicVoltage = MotionMagicVoltage(0.0, false, 0.0, 0, false, false, false)
    private val m_voltageOutControl: VoltageOut = VoltageOut(0.0)

    private var m_targetVoltage: Double = 0.0
    private var m_driveType: Boolean = true

    constructor(driveType: Boolean) {
        m_driveType = driveType
    }

    constructor() {
        m_driveType = true
    }

    override fun apply(parameters: SwerveControlRequestParameters, vararg modulesToApply: SwerveModule): StatusCode {
        for (module: SwerveModule in modulesToApply) {
            if (m_driveType) {
                // Command steer motor to zero
                module.getSteerMotor().setControl(m_motionMagicControl)

                // Command drive motor to voltage
                module.getDriveMotor().setControl(m_voltageOutControl.withOutput(m_targetVoltage))
            } else {
                // Command steer motor to voltage
                module.getSteerMotor().setControl(m_voltageOutControl.withOutput(m_targetVoltage))

                // Command drive motor to zero
                module.getDriveMotor().setControl(m_motionMagicControl)
            }
        }

        return StatusCode.OK
    }

    /**
     *
     * @param targetVoltage Voltage for all modules to target
     * @return
     */
    fun withVoltage(targetVoltage: Double): SwerveVoltageRequest {
        this.m_targetVoltage = targetVoltage
        return this
    }
}
