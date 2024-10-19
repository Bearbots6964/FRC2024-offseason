// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.ctre.phoenix6.SignalLogger
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler

class Robot : TimedRobot() {
    private var m_autonomousCommand: Command? = null

    private var m_robotContainer: RobotContainer? = null

    override fun robotInit() {
        m_robotContainer = RobotContainer()

        m_robotContainer!!.drivetrain.daqThread.setThreadPriority(99)


        DriverStation.silenceJoystickConnectionWarning(true)
        SignalLogger.start()
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
    }

    override fun disabledInit() {}

    override fun disabledPeriodic() {}

    override fun disabledExit() {}

    override fun autonomousInit() {
        m_autonomousCommand = m_robotContainer?.autonomousCommand

        if (m_autonomousCommand != null) {
            m_autonomousCommand!!.schedule()
        }
    }

    override fun autonomousPeriodic() {}

    override fun autonomousExit() {}

    override fun teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand!!.cancel()
        }
    }

    override fun teleopPeriodic() {}

    override fun teleopExit() {}

    override fun testInit() {
        CommandScheduler.getInstance().cancelAll()
    }

    override fun testPeriodic() {}

    override fun testExit() {}

    override fun simulationPeriodic() {}
}
