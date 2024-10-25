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
import frc.robot.Util.LocalADStarAK
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGWriter

class Robot : LoggedRobot() {
    private var m_autonomousCommand: Command? = null

    private var m_robotContainer: RobotContainer? = null

    override fun robotInit() {
        Pathfinding.setPathfinder(LocalADStarAK())

        Logger.recordMetadata("ProjectName", "FRC2024-offseason") // Set a metadata value

//        if (isReal()) {
        Logger.addDataReceiver(WPILOGWriter()) // Log to a USB stick ("/U/logs") TODO get usb stick
        Logger.addDataReceiver(NT4Publisher()) // Publish data to NetworkTables
        PowerDistribution(1, PowerDistribution.ModuleType.kRev) // Enables power distribution logging
//        }
//        else {
//            setUseTiming(false) // Run as fast as possible
//            val logPath = LogFileUtil.findReplayLog() // Pull the replay log from AdvantageScope (or prompt the user)
//            Logger.setReplaySource(WPILOGReader(logPath)) // Read replay log
//            Logger.addDataReceiver(
//                WPILOGWriter(
//                    LogFileUtil.addPathSuffix(
//                        logPath,
//                        "_sim",
//                    ),
//                ),
//            ) // Save outputs to a new log
//        }

// Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow" page
        Logger.start() // Start logging! No more data receivers, replay sources, or metadata values may be added.
        m_robotContainer = RobotContainer()

        m_robotContainer!!.drivetrain.daqThread.setThreadPriority(99)

        DriverStation.silenceJoystickConnectionWarning(true)
        SignalLogger.start()

        PathfindingCommand.warmupCommand().schedule()
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
    }

    override fun disabledInit() {}

    override fun disabledPeriodic() {}

    override fun disabledExit() {}

    override fun autonomousInit() {
        m_autonomousCommand = m_robotContainer?.autoCommand

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
