// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState
import com.ctre.phoenix6.swerve.SwerveModule
import com.ctre.phoenix6.swerve.SwerveRequest
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.commands.GoToNoteCommand
import frc.robot.commands.PutArmDownCommand
import frc.robot.generated.TunerConstants
import frc.robot.generated.TunerConstants.createDrivetrain
import frc.robot.subsystems.ArmSubsystem
import frc.robot.subsystems.drive.CommandSwerveDrivetrain
import frc.robot.subsystems.vision.VisionSubsystem

class RobotContainer {
    private val MaxSpeed =
        TunerConstants.kSpeedAt12VoltsMps.`in`(Units.MetersPerSecond) // kSpeedAt12Volts desired top speed
    private val MaxAngularRate = Units.RotationsPerSecond.of(0.75)
        .`in`(Units.RadiansPerSecond) // 3/4 of a rotation per second max angular velocity

    private val joystick = CommandXboxController(0)

    private val logger = Telemetry(MaxSpeed)

    @JvmField
    val drivetrain: CommandSwerveDrivetrain = createDrivetrain()

    val armSubsystem: ArmSubsystem = ArmSubsystem()
    val visionSubsystem: VisionSubsystem = VisionSubsystem.instance

    val goToNoteCommand = GoToNoteCommand(drivetrain, visionSubsystem)
    val putArmDownCommand = PutArmDownCommand(armSubsystem)

    /* Setting up bindings for necessary control of the swerve drive platform */
    private val drive: SwerveRequest.FieldCentric = SwerveRequest.FieldCentric().withDeadband(MaxSpeed * 0.1)
        .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
    private val brake = SwerveRequest.SwerveDriveBrake()
    private val point = SwerveRequest.PointWheelsAt()
    private val forwardStraight: SwerveRequest.RobotCentric =
        SwerveRequest.RobotCentric().withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)

    /* Path follower */
    private val autoChooser: SendableChooser<Command>

    init {
        configureBindings()

        autoChooser = AutoBuilder.buildAutoChooser("Tests")
        SmartDashboard.putData("Auto Mode", autoChooser)

        // named commands
        NamedCommands.registerCommands(
            mapOf(
                "extendArm" to Commands.run({ armSubsystem.extend(1.0) }, armSubsystem),
                "retractArm" to Commands.run({ armSubsystem.retract(1.0) }, armSubsystem),
            ),
        )
    }

    private fun configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.defaultCommand = drivetrain.applyRequest {
            drive.withVelocityX(-joystick.leftY * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(-joystick.leftX * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(-joystick.rightX * MaxAngularRate)
        } // Drive counterclockwise with negative X (left)

        armSubsystem.defaultCommand = armSubsystem.getCommand {
            MathUtil.applyDeadband(
                joystick.rightTriggerAxis - joystick.leftTriggerAxis, 0.1,
            )
        }

        joystick.a().whileTrue(drivetrain.applyRequest { brake })
        joystick.b().whileTrue(
            drivetrain.applyRequest {
                point.withModuleDirection(
                    Rotation2d(
                        -joystick.leftY,
                        -joystick.leftX,
                    ),
                )
            },
        )

        joystick.pov(0).whileTrue(
            drivetrain.applyRequest { forwardStraight.withVelocityX(0.5).withVelocityY(0.0) },
        )
        joystick.pov(180).whileTrue(
            drivetrain.applyRequest { forwardStraight.withVelocityX(-0.5).withVelocityY(0.0) },
        )

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward))
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse))
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward))
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse))

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce { drivetrain::resetPose })

        joystick.rightBumper().whileTrue(goToNoteCommand.andThen(putArmDownCommand))

        drivetrain.registerTelemetry { state: SwerveDriveState? ->
            if (state != null) {
                logger.telemeterize(state)
            }
        }
    }

    val autonomousCommand: Command
        get() = /* First put the drivetrain into auto run mode, then run the auto */
            autoChooser.selected
}
