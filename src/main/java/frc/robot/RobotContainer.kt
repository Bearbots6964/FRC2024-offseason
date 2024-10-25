// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot

import com.ctre.phoenix6.Utils
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest
import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.generated.TunerConstants
import frc.robot.subsystems.ArmSubsystem
import frc.robot.subsystems.drive.CommandSwerveDrivetrain
import frc.robot.subsystems.vision.VisionSubsystem
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import java.util.function.Supplier
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin

class RobotContainer {
    /** Defaults to true top speed. */
    private var maxSpeed: Double = TunerConstants.kSpeedAt12VoltsMps
    /** Reduction in speed from Max Speed, 0.1 = 10% */
    private val turtleSpeed = 0.1
    /**
     * .75 rotation per second max angular velocity. Adjust for max
     * turning rate speed.
     */
    private val maxAngularRate = Math.PI * 1.5
    /**
     * .75 rotation per second max angular velocity. Adjust for max
     * turning rate speed.
     */
    private val turtleAngularRate = Math.PI * 0.5
    /** This will be updated when turtle and reset to MaxAngularRate */
    private var angularRate = maxAngularRate
    // Bindings for necessary control of the swerve drive platform
    /** Driver xbox controller. */
    private var drv: CommandXboxController = CommandXboxController(0)
    /** Operator xbox controller. */
    private var op: CommandXboxController = CommandXboxController(1)
    var drivetrain: CommandSwerveDrivetrain = TunerConstants.DriveTrain
    
    /**
     * Field-centric driving in Open Loop, can change to closed
     * loop after characterization.
     *
     * For closed loop replace [DriveRequestType.OpenLoopVoltage]
     * with [DriveRequestType.Velocity].
     */
    private var drive: SwerveRequest.FieldCentric =
        SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withDeadband(maxSpeed * 0.1) // Deadband is handled on input
            .withRotationalDeadband(angularRate * 0.1)
    private var brake: SwerveRequest.SwerveDriveBrake = SwerveRequest.SwerveDriveBrake()
    private var point: SwerveRequest.PointWheelsAt = SwerveRequest.PointWheelsAt()
    private var logger: Telemetry = Telemetry(maxSpeed)
    private var controlStyle: Supplier<SwerveRequest?>? = null
    private var lastControl = "2 Joysticks"
    private var lastSpeed = 0.65
    
    private val armSubsystem: ArmSubsystem = ArmSubsystem
    private val visionSubsystem: VisionSubsystem = VisionSubsystem(drivetrain)
    var vision: VisionSubsystem = VisionSubsystem(drivetrain)
    
    // dashboard options
    
    private val autoChooser: LoggedDashboardChooser<Command>
    private val controlChooser: SendableChooser<String> = SendableChooser()
    private val controlChooserLogger: LoggedDashboardChooser<String>
    private val speedChooser: SendableChooser<Double> = SendableChooser()
    private val speedChooserLogger: LoggedDashboardChooser<Double>
    
    val autoCommand: Command?
        get() = // First put the drivetrain into auto run mode, then run the auto
            autoChooser.get()
    
    private fun configureBindings() {
        SmartDashboard.putData("Auto Chooser", autoChooser.sendableChooser)
        
        newControlStyle()
        newSpeed()
        
        armSubsystem.defaultCommand =
            armSubsystem.getCommand { op.rightTriggerAxis - op.leftTriggerAxis }
        
        drv.a().whileTrue(drivetrain.applyRequest { brake })
        drv.b()
            .whileTrue(
                drivetrain.applyRequest {
                    point.withModuleDirection(
                        Rotation2d(
                            -drv.leftY,
                            -drv.leftX,
                        ),
                    )
                },
            )
        // reset the field-centric heading on start button press
        drv.start().onTrue(drivetrain.runOnce { drivetrain.seedFieldRelative() })
        // Turtle Mode while held
        drv.leftBumper()
            .onTrue(
                Commands.runOnce({
                    maxSpeed = TunerConstants.kSpeedAt12VoltsMps * turtleSpeed
                })
                    .andThen({ angularRate = turtleAngularRate }),
            )
        drv.leftBumper()
            .onFalse(
                Commands.runOnce({
                    maxSpeed =
                        TunerConstants.kSpeedAt12VoltsMps *
                            speedChooserLogger.get()
                })
                    .andThen({ angularRate = maxAngularRate }),
            )
        
        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(Pose2d(Translation2d(), Rotation2d.fromDegrees(0.0)))
        }
        
        drivetrain.registerTelemetry { state: SwerveDrivetrain.SwerveDriveState ->
            logger.telemeterize(state)
        }
        val controlPick = Trigger { lastControl !== controlChooserLogger.get() }
        controlPick.onTrue(Commands.runOnce({ newControlStyle() }))
        val speedPick = Trigger {
            lastSpeed !=
                try {
                    speedChooserLogger.get()!!
                } catch (e: Throwable) {
                    0.65
                }
        }
        speedPick.onTrue(Commands.runOnce({ newSpeed() }))
        
        drv.x()
            .and(drv.pov(0))
            .whileTrue(drivetrain.runDriveQuasiTest(SysIdRoutine.Direction.kForward))
        drv.x()
            .and(drv.pov(180))
            .whileTrue(drivetrain.runDriveQuasiTest(SysIdRoutine.Direction.kReverse))
        
        drv.y()
            .and(drv.pov(0))
            .whileTrue(drivetrain.runDriveDynamTest(SysIdRoutine.Direction.kForward))
        drv.y()
            .and(drv.pov(180))
            .whileTrue(drivetrain.runDriveDynamTest(SysIdRoutine.Direction.kReverse))
        
        drv.a()
            .and(drv.pov(0))
            .whileTrue(drivetrain.runSteerQuasiTest(SysIdRoutine.Direction.kForward))
        drv.a()
            .and(drv.pov(180))
            .whileTrue(drivetrain.runSteerQuasiTest(SysIdRoutine.Direction.kReverse))
        
        drv.b()
            .and(drv.pov(0))
            .whileTrue(drivetrain.runSteerDynamTest(SysIdRoutine.Direction.kForward))
        drv.b()
            .and(drv.pov(180))
            .whileTrue(drivetrain.runSteerDynamTest(SysIdRoutine.Direction.kReverse))
        
        // Drivetrain needs to be placed against a sturdy wall and test stopped immediately upon
        // wheel slip
        drv.back().and(drv.pov(0)).whileTrue(drivetrain.runDriveSlipTest())
        drv.rightBumper().whileTrue(drivetrain.rotateToAngleDiff(visionSubsystem.getNoteCamAngle()))
        
        // TODO make sure this works and probably also see if we can't get it to also move the robot forward a bit to center it w/ the arm
    }
    
    init {
        // Detect if controllers are missing / stop multiple warnings
        DriverStation.silenceJoystickConnectionWarning(true)
        
        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = LoggedDashboardChooser("Auto Chooser", AutoBuilder.buildAutoChooser())
        
        controlChooser.addOption("1 Joystick Rotation Triggers", "1 Joystick Rotation Triggers")
        controlChooser.addOption(
            "Split Joysticks Rotation Triggers",
            "Split Joysticks Rotation Triggers",
        )
        controlChooser.addOption("2 Joysticks with Gas Pedal", "2 Joysticks with Gas Pedal")
        controlChooser.setDefaultOption("2 Joysticks", "2 Joysticks")
        controlChooserLogger = LoggedDashboardChooser("Control Chooser", controlChooser)
        
        speedChooser.addOption("100%", 1.0)
        speedChooser.addOption("95%", 0.95)
        speedChooser.addOption("90%", 0.9)
        speedChooser.addOption("85%", 0.85)
        speedChooser.addOption("80%", 0.8)
        speedChooser.addOption("75%", 0.75)
        speedChooser.addOption("70%", 0.7)
        speedChooser.setDefaultOption("65%", 0.65)
        speedChooser.addOption("60%", 0.6)
        speedChooser.addOption("55%", 0.55)
        speedChooser.addOption("50%", 0.5)
        speedChooser.addOption("35%", 0.35)
        speedChooserLogger = LoggedDashboardChooser("Speed Chooser", speedChooser)
        
        configureBindings()
    }
    
    private fun newControlStyle() {
        lastControl =
            if (controlChooserLogger.get() != null) controlChooserLogger.get()
            else "2 Joysticks"
        
        when (controlChooserLogger.get()) {
            "2 Joysticks" ->
                controlStyle =
                    Supplier<SwerveRequest?> {
                        // Drive forward -Y
                        drive.withVelocityX(-drv.leftY * maxSpeed)
                            // Drive left with negative X (left)
                            .withVelocityY(
                                -drv.leftX * maxSpeed
                            )
                            // Drive counterclockwise with negative X (left)
                            .withRotationalRate(-drv.rightX * angularRate)
                    }
            
            "1 Joystick Rotation Triggers" ->
                controlStyle =
                    Supplier<SwerveRequest?> {
                        // Drive forward -Y
                        drive.withVelocityX(-drv.leftY * maxSpeed)
                            // Drive left with negative X (left)
                            .withVelocityY(
                                -drv.leftX * maxSpeed
                            )
                            .withRotationalRate(
                                (drv.leftTriggerAxis - drv.rightTriggerAxis) *
                                    angularRate
                            )
                    }
            
            "Split Joysticks Rotation Triggers" ->
                controlStyle =
                    Supplier<SwerveRequest?> {
                        // Left stick forward/back
                        drive.withVelocityX(
                            -drv.leftY * maxSpeed
                        )
                            // Right stick strafe
                            .withVelocityY(-drv.rightX * maxSpeed)
                            .withRotationalRate(
                                (drv.leftTriggerAxis - drv.rightTriggerAxis) *
                                    angularRate
                            )
                    }
            
            "2 Joysticks with Gas Pedal" ->
                controlStyle =
                    Supplier<SwerveRequest?> {
                        val stickX: Double = -drv.leftX
                        val stickY: Double = -drv.leftY
                        val angle: Double = atan2(stickX, stickY)
                        
                        // left x * gas
                        drive.withVelocityX(
                            cos(angle) * drv.rightTriggerAxis * maxSpeed
                        )
                            // Angle of left stick Y * gas pedal
                            .withVelocityY(
                                sin(angle) * drv.rightTriggerAxis * maxSpeed
                            )
                            // Drive counterclockwise with negative X (left)
                            .withRotationalRate(
                                -drv.rightX * angularRate
                            )
                    }
            
            else -> {
                DriverStation.reportWarning(
                    "RC: Invalid control style: " + controlChooserLogger.get(),
                    false
                )
                controlStyle =
                    Supplier<SwerveRequest?> {
                        // Drive forward -Y
                        drive.withVelocityX(-drv.leftY * maxSpeed)
                            // Drive left with negative X (left)
                            .withVelocityY(
                                -drv.leftX * maxSpeed
                            )
                            .withRotationalRate(-drv.rightX * angularRate)
                    }
            }
        }
        try {
            drivetrain.defaultCommand.cancel()
        } catch (_: Exception) {
            DriverStation.reportWarning(
                "RC: Failed to cancel default command? Maybe something else...",
                true
            )
            
            controlStyle =
                Supplier<SwerveRequest?> {
                    // Drive forward -Y
                    drive.withVelocityX(-drv.leftY * maxSpeed)
                        // Drive left with negative X (left)
                        .withVelocityY(
                            -drv.leftX * maxSpeed
                        )
                        .withRotationalRate(-drv.rightX * angularRate)
                }
        }
        drivetrain.defaultCommand = drivetrain.applyRequest(controlStyle!!).ignoringDisable(true)
    }
    
    private fun newSpeed() {
        lastSpeed = try {
            speedChooserLogger.get()
        } catch (_: Exception) {
            DriverStation.reportWarning(
                "RC: Failed to get speed, value is: " + speedChooserLogger.get(),
                false
            )
            0.65
        }
        maxSpeed = TunerConstants.kSpeedAt12VoltsMps * lastSpeed
    }
}
