// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler

class Robot : TimedRobot() {
    private var autonomousCommand: Command? = null

    private var robotContainer: RobotContainer? = null

    override fun robotInit() {
        robotContainer = RobotContainer()
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
    }

    override fun disabledInit() {}

    override fun disabledPeriodic() {}

    override fun disabledExit() {}

    override fun autonomousInit() {
        autonomousCommand = robotContainer?.autonomousCommand

        if (autonomousCommand != null) {
            autonomousCommand!!.schedule()
        }
    }

    override fun autonomousPeriodic() {}

    override fun autonomousExit() {}

    override fun teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand!!.cancel()
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
