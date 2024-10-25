package frc.robot.commands

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.ArmSubsystem

class PutArmDownCommand(private val armSubsystem: ArmSubsystem) : Command() {

    var maxLimit = 5.0

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(armSubsystem)
        SmartDashboard.putNumber("Arm Max Current Limit", maxLimit)
    }

    override fun initialize() {}

    override fun execute() {
        armSubsystem.extend(1.0)
        if (SmartDashboard.getNumber("Arm Max Current Limit", maxLimit) != maxLimit)
            maxLimit = SmartDashboard.getNumber("Arm Max Current Limit", maxLimit)
    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return armSubsystem.getOutputCurrent() > maxLimit
    }

    override fun end(interrupted: Boolean) {
        armSubsystem.stop()
    }
}
