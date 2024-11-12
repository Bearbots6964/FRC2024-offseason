// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.pathplanner.lib.commands.PathfindingCommand
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler

class Robot : TimedRobot() {
    private var m_autonomousCommand: Command? = null

    private var m_robotContainer: RobotContainer? = null

    init {
//        if (isReal()) {
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
        m_robotContainer = RobotContainer()

        // m_robotContainer!!.drivetrain.OdometryThread().setThreadPriority(Thread.MAX_PRIORITY)

        DriverStation.silenceJoystickConnectionWarning(true)

        PathfindingCommand.warmupCommand().schedule()

        SmartDashboard.putData(CommandScheduler.getInstance())
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
        m_robotContainer!!.updatePoseEstimation()
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

    companion object {
        fun sim(): Boolean {
            return isSimulation()
        }
    }
}
