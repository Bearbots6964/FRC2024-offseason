package frc.robot.commands

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType
import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.generated.TunerConstants
import frc.robot.subsystems.drive.CommandSwerveDrivetrain
import frc.robot.subsystems.vision.VisionSubsystem
import kotlin.math.PI

class GoToNoteCommand(
    private val commandSwerveDrivetrain: CommandSwerveDrivetrain,
    private val visionSubsystem: VisionSubsystem,
) : Command() {

    var drive: SwerveRequest.RobotCentric =
        SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.Velocity)
            .withDeadband(0.0) // Deadband is handled on input
            .withRotationalDeadband(0.0)

    val anglePid = PIDController(0.03, 0.0, 0.001)
    val drivePid = PIDController(0.03, 0.0, 0.001)

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(commandSwerveDrivetrain, visionSubsystem)
        SmartDashboard.putData("Angle PID", anglePid)
        SmartDashboard.putData("Drive PID", drivePid)
    }

    override fun initialize() {
    }

    override fun execute() {
        try {
            commandSwerveDrivetrain.setControl(
                drive.withVelocityX(-drivePid.calculate(visionSubsystem.getNoteCamPitch() * TunerConstants.kSpeedAt12VoltsMps.`in`(
                    Units.MetersPerSecond)))
                    .withVelocityY(0.0).withRotationalDeadband(0.1).withDeadband(1.0)
                    .withRotationalRate(anglePid.calculate(visionSubsystem.getNoteCamYaw()) * 2 * PI)
            )
        } finally {}


    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        try {
            return visionSubsystem.isCloseEnoughToDone()
        } finally {
            return false
        }
    }

    override fun end(interrupted: Boolean) {

    }
}
