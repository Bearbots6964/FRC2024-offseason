// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.ctre.phoenix6.Utils
import frc.robot.bad_code_stupid.SwerveDrivetrain.SwerveDriveState
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.*
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.generated.TunerConstants
import frc.robot.subsystems.CommandSwerveDrivetrain

class RobotContainer {
    private val maxSpeed = TunerConstants.speedAt12VoltsMps // kSpeedAt12VoltsMps desired top speed
    private val maxAngularRate = 1.5 * Math.PI // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private val joystick = CommandXboxController(0) // My joystick
    private val drivetrain: CommandSwerveDrivetrain = TunerConstants.DriveTrain // My drivetrain

    private val drive: FieldCentric = FieldCentric()
        .withDeadband(maxSpeed * 0.1).withRotationalDeadband(maxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // I want field-centric

    // driving in open loop
    private val brake = SwerveDriveBrake()
    private val point = PointWheelsAt()

    private val logger = Telemetry(maxSpeed)

    private fun configureBindings() {
        drivetrain.defaultCommand = drivetrain.applyRequest {
            drive.withVelocityX(-joystick.leftY * maxSpeed) // Drive forward with
                // negative Y (forward)
                .withVelocityY(-joystick.leftX * maxSpeed) // Drive left with negative X (left)
                .withRotationalRate(-joystick.rightX * maxAngularRate)
        } // Drive counterclockwise with negative X (left)


        joystick.a().whileTrue(drivetrain.applyRequest { brake })
        joystick.b().whileTrue(drivetrain
            .applyRequest { point.withModuleDirection(Rotation2d(-joystick.leftY, -joystick.leftX)) })

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce { drivetrain.seedFieldRelative() })

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(Pose2d(Translation2d(), Rotation2d.fromDegrees(90.0)))
        }
        drivetrain.registerTelemetry { state: SwerveDriveState -> logger.telemeterize(state) }
    }

    init {
        configureBindings()
    }

    val autonomousCommand: Command
        get() = Commands.print("No autonomous command configured")
}
