// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.ctre.phoenix6.Utils
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.*
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.generated.TunerConstants
import frc.robot.subsystems.CommandSwerveDrivetrain
import frc.robot.subsystems.VisionSubsystem
import java.util.function.Supplier

class RobotContainer {
    private val autoChooser: SendableChooser<Command>? = null
    private val controlChooser = SendableChooser<String>()
    private val speedChooser = SendableChooser<Double>()
    private val MaxSpeed = TunerConstants.kSpeedAt12VoltsMps // Initial max is true top speed
    private val TurtleSpeed = 0.1 // Reduction in speed from Max Speed, 0.1 = 10%
    private val MaxAngularRate =
        Math.PI * 1.5 // .75 rotation per second max angular velocity.  Adjust for max turning rate speed.
    private val TurtleAngularRate =
        Math.PI * 0.5 // .75 rotation per second max angular velocity.  Adjust for max turning rate speed.
    private val AngularRate = MaxAngularRate // This will be updated when turtle and reset to MaxAngularRate

    /* Setting up bindings for necessary control of the swerve drive platform */
    private val joystick = CommandXboxController(0)
    var drivetrain: CommandSwerveDrivetrain = TunerConstants.DriveTrain // drivetrain
    val camera : VisionSubsystem = VisionSubsystem()

    // Slew Rate Limiters to limit acceleration of joystick inputs
    private val xLimiter = SlewRateLimiter(2.0)
    private val yLimiter = SlewRateLimiter(0.5)
    private val rotLimiter = SlewRateLimiter(0.5)

    // Field-centric driving in Open Loop, can change to closed loop after characterization
    // For closed loop replace DriveRequestType.OpenLoopVoltage with DriveRequestType.Velocity
    var drive: FieldCentric = FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withDeadband(MaxSpeed * 0.1) // Deadband is handled on input
        .withRotationalDeadband(AngularRate * 0.1)

    var brake: SwerveDriveBrake = SwerveDriveBrake()
    var forwardStraight: RobotCentric = RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage)
    var point: PointWheelsAt = PointWheelsAt()

    private val logger = Telemetry(MaxSpeed)

    var odomStart: Pose2d = Pose2d(0.0, 0.0, Rotation2d(0.0, 0.0))

    private val controlStyle: Supplier<SwerveRequest>? = null

    private val lastControl = "2 Joysticks"
    private val lastSpeed = 0.65

    private fun configureBindings() {
        drivetrain.defaultCommand = drivetrain.applyRequest {
            drive.withVelocityX(-joystick.leftY * MaxSpeed) // Drive forward with
                // negative Y (forward)
                .withVelocityY(-joystick.leftX * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(-joystick.rightX * MaxAngularRate)
        } // Drive counterclockwise with negative X (left)


        joystick.a().whileTrue(drivetrain.applyRequest { brake })
        joystick.b().whileTrue(drivetrain
            .applyRequest { point.withModuleDirection(Rotation2d(-joystick.leftY, -joystick.leftX)) })

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce { drivetrain.seedFieldRelative() })

        //face robot toward closest note
        joystick.rightBumper().onTrue(drivetrain.applyRequest { point.withModuleDirection(camera.getNearestRotation())})

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(Pose2d(Translation2d(), Rotation2d.fromDegrees(90.0)))
        }
        drivetrain.registerTelemetry { state: SwerveDrivetrain.SwerveDriveState -> logger.telemeterize(state) }
    }



    init {
        configureBindings()
    }

    val autonomousCommand: Command
        get() = Commands.print("No autonomous command configured")
}
