// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot

import com.ctre.phoenix6.SignalLogger
import com.pathplanner.lib.commands.PathfindingCommand
import com.pathplanner.lib.pathfinding.Pathfinding
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.util.LocalADStarAK
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGWriter

class Robot : LoggedRobot() {
    private var robotContainer: RobotContainer = RobotContainer()
    private var autonomousCommand: Command? = null

    override fun robotInit() {
        Logger.recordMetadata("ProjectName", "FRC2024-offseason")
        // Log to a USB stick ("/U/logs")
        Logger.addDataReceiver(WPILOGWriter()) // TODO get usb stick
        // Publish data to NetworkTables
        Logger.addDataReceiver(NT4Publisher())
        // Enables power distribution logging
        PowerDistribution(
            1,
            PowerDistribution.ModuleType.kRev,
        )

        // Start logging! No more data receivers, replay sources, or metadata values may be added.
        Logger.start()
        SignalLogger.start()

        DriverStation.silenceJoystickConnectionWarning(true)
        robotContainer.drivetrain.daqThread.setThreadPriority(99)

        Pathfinding.setPathfinder(LocalADStarAK())
        PathfindingCommand.warmupCommand().schedule()
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
    }

    override fun disabledInit() {}
    override fun disabledPeriodic() {}
    override fun disabledExit() {}

    override fun autonomousInit() {
        autonomousCommand = robotContainer.autoCommand

        if (autonomousCommand != null) {
            autonomousCommand?.schedule()
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
