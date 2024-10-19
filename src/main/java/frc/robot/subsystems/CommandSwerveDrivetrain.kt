package frc.robot.subsystems

import com.ctre.phoenix6.Utils
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import java.util.function.Supplier

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
@Suppress("ConstPropertyName")
class CommandSwerveDrivetrain : SwerveDrivetrain, Subsystem {
    private var simNotifier: Notifier? = null
    private var lastSimTime = 0.0

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private val blueAlliancePerspectiveRotation: Rotation2d = Rotation2d.fromDegrees(0.0)

    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private val redAlliancePerspectiveRotation: Rotation2d = Rotation2d.fromDegrees(180.0)

    /* Keep track if we've ever applied the operator perspective before or not */
    private var hasAppliedOperatorPerspective = false

    constructor(
        driveTrainConstants: SwerveDrivetrainConstants,
        odometryUpdateFrequency: Double,
        vararg modules: SwerveModuleConstants?,
    ) : super(driveTrainConstants, odometryUpdateFrequency, *modules) {

    }

    constructor(driveTrainConstants: SwerveDrivetrainConstants, vararg modules: SwerveModuleConstants?) : super(
        driveTrainConstants,
        *modules,
    ) {

    }

    fun applyRequest(requestSupplier: Supplier<SwerveRequest?>): Command {
        return run { this.setControl(requestSupplier.get()) }
    }



    override fun periodic() {
        /* Periodically try to apply the operator perspective */
        /* If we haven't applied the operator perspective before, then we should apply it regardless of DS state */
        /* This allows us to correct the perspective in case the robot code restarts mid-match */
        /* Otherwise, only check and apply the operator perspective if the DS is disabled */
        /* This ensures driving behavior doesn't change until an explicit disable event occurs during testing*/
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent { allianceColor: Alliance ->
                this.setOperatorPerspectiveForward(
                    if (allianceColor == Alliance.Red)
                        redAlliancePerspectiveRotation
                    else
                        blueAlliancePerspectiveRotation
                )
                hasAppliedOperatorPerspective = true
            }
        }
    }

    companion object {
        private const val simLoopPeriod = 0.005 // 5 ms
    }
}
