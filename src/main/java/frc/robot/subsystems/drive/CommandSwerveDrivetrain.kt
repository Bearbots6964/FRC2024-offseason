package frc.robot.subsystems.drive

import com.ctre.phoenix6.Utils
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest
import com.ctre.phoenix6.signals.NeutralModeValue
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.commands.PathPlannerAuto
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.path.PathPlannerPath
import com.pathplanner.lib.util.HolonomicPathFollowerConfig
import com.pathplanner.lib.util.PIDConstants
import com.pathplanner.lib.util.ReplanningConfig
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.Voltage
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.generated.TunerConstants
import frc.robot.util.ModifiedSignalLogger
import frc.robot.util.SwerveVoltageRequest
import org.littletonrobotics.junction.AutoLogOutput
import java.util.*
import java.util.function.Supplier
import kotlin.math.PI
import kotlin.math.max

/**
 * Class that extends the Phoenix SwerveDrivetrain class and
 * implements subsystem so it can be used in command-based
 * projects easily.
 */
class CommandSwerveDrivetrain : SwerveDrivetrain, Subsystem {
    private val autoRequest: SwerveRequest.ApplyChassisSpeeds = SwerveRequest.ApplyChassisSpeeds()
    private var driveBaseRadius = 0.0
    private var simNotifier: Notifier? = null
    private var lastSimTime = 0.0
    private var thetaController: PIDController = PIDController(1.1, 0.0, 0.125) // TODO tune this
    private var angleJoystickDeadbandRadius = 0.5
    
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
        configNeutralMode(NeutralModeValue.Coast)
        for (moduleLocation in m_moduleLocations) {
            driveBaseRadius = max(driveBaseRadius, moduleLocation.norm)
            if (Utils.isSimulation()) {
                startSimThread()
            }
        }
        configurePathPlanner()
        
        SmartDashboard.putData(thetaController) // for tuning
    }
    
    constructor(
        driveTrainConstants: SwerveDrivetrainConstants,
        vararg modules: SwerveModuleConstants?,
    ) : super(
        driveTrainConstants,
        *modules,
    ) {
        configNeutralMode(NeutralModeValue.Coast)
        configurePathPlanner()
        if (Utils.isSimulation()) {
            startSimThread()
        }
    }
    
    private fun configurePathPlanner() {
        AutoBuilder.configureHolonomic(
            { this.state.Pose }, // Supplier of current robot pose
            this::seedFieldRelative, // Consumer for seeding pose against auto
            { this.currentRobotChassisSpeeds },
            { speeds -> this.setControl(autoRequest.withSpeeds(speeds)) }, // Consumer of ChassisSpeeds to drive the robot
            holonomicPathFollowerConfig(),
            getAlliance(),
            this,
        ) // Subsystem for requirements
    }
    
    private fun holonomicPathFollowerConfig() = HolonomicPathFollowerConfig(
        PIDConstants(10.0, 0.0, 0.0),
        PIDConstants(10.0, 0.0, 0.0),
        TunerConstants.kSpeedAt12VoltsMps,
        driveBaseRadius,
        ReplanningConfig(),
    )
    
    fun applyRequest(requestSupplier: Supplier<SwerveRequest?>): Command {
        return run { this.setControl(requestSupplier.get()) }
    }
    
    private fun startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds()
        
        /* Run simulation at a faster rate so PID gains behave more reasonably */
        simNotifier = Notifier {
            val currentTime = Utils.getCurrentTimeSeconds()
            val deltaTime = currentTime - lastSimTime
            lastSimTime = currentTime
            
            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage())
        }
        simNotifier!!.startPeriodic(SIM_LOOP_PERIOD)
    }
    
    fun getAutoPath(pathName: String?): Command {
        return PathPlannerAuto(pathName)
    }
    
    fun driveToPose(pose: Pose2d): Command {
        // Create the constraints to use during pathfinding
        val constraints = PathConstraints(9.46, 2.0, 2 * PI, 4 * PI)
        
        return AutoBuilder.pathfindToPose(pose, constraints, 0.0, 0.0)
    }
    
    fun pathfindThenFollowPath(pathName: String): Command {
        val path = PathPlannerPath.fromPathFile(pathName)
        // Create the constraints to use during pathfinding
        val constraints = PathConstraints(9.46, 2.0, 2 * PI, 4 * PI)
        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        return AutoBuilder.pathfindThenFollowPath(path, constraints, 0.0)
    }
    
    fun rotateToAngleDiff(angle: Double): Command {
        return run {
            // first get the current angle
            val currentAngle = state.Pose.rotation.degrees
            // then get the difference between the current angle and the target angle
            val angleDifference =
                Units.Degree.of(angle - currentAngle).`in`(Units.Radian) // convert to radians
            applyRequest {
                SwerveRequest.FieldCentric()
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                    .withRotationalRate(
                        thetaController.calculate(
                            angleDifference,
                            state.Pose.rotation.radians
                        ) * 2 * PI
                    )
            }
        }
    }
    
    var constraints = PathConstraints(9.46, 2.0, 2 * PI, 4 * PI)
        get() {
            return field
        }
    
    fun useSpeeds(speeds: ChassisSpeeds) {
        autoRequest.withSpeeds(speeds)
    }
    
    private fun getHolonomicDriveController() = PPHolonomicDriveController(
        PIDConstants(10.0, 0.0, 0.0),
        PIDConstants(10.0, 0.0, 0.0),
        // max module speed is 9.46 m/s, convert to ft/s
        FeetPerSecond.of(9.46).`in`(MetersPerSecond),
        driveBaseRadius,
        
        )
    
    private fun getAlliance() = alliance@{
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field during auto only.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        val alliance: Optional<DriverStation.Alliance> = DriverStation.getAlliance()
        if (alliance.isPresent) {
            return@alliance (alliance.get() == DriverStation.Alliance.Red) and !DriverStation.isTeleop()
        }
        false
    }
    
    private fun getReplanningConfig() = ReplanningConfig(true, true)
    
    override fun simulationPeriodic() {
        /* Assume 20ms update rate, get battery voltage from WPILib */
        updateSimState(0.02, RobotController.getBatteryVoltage())
    }
    
    @get:AutoLogOutput
    val currentRobotChassisSpeeds: ChassisSpeeds
        get() = m_kinematics.toChassisSpeeds(*state.ModuleStates)
    
    private val driveVoltageRequest: SwerveVoltageRequest = SwerveVoltageRequest(true)
    
    private val m_driveSysIdRoutine: SysIdRoutine = SysIdRoutine(
        SysIdRoutine.Config(null, null, null, ModifiedSignalLogger.logState()),
        SysIdRoutine.Mechanism(
            { volts: Measure<Voltage?> ->
                setControl(
                    driveVoltageRequest.withVoltage(
                        volts.`in`(
                            Volts,
                        ),
                    ),
                )
            },
            null,
            this,
        ),
    )
    
    private val steerVoltageRequest: SwerveVoltageRequest = SwerveVoltageRequest(false)
    
    private val m_steerSysIdRoutine: SysIdRoutine = SysIdRoutine(
        SysIdRoutine.Config(null, null, null, ModifiedSignalLogger.logState()),
        SysIdRoutine.Mechanism(
            { volts: Measure<Voltage?> ->
                setControl(
                    steerVoltageRequest.withVoltage(
                        volts.`in`(
                            Volts,
                        ),
                    ),
                )
            },
            null,
            this,
        ),
    )
    
    override fun periodic() {
        /* Periodically try to apply the operator perspective */
        /* If we haven't applied the operator perspective before, then we should apply it regardless of DS state */
        /* This allows us to correct the perspective in case the robot code restarts mid-match */
        /* Otherwise, only check and apply the operator perspective if the DS is disabled */
        /* This ensures driving behavior doesn't change until an explicit disable event occurs during testing*/
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent { allianceColor: Alliance ->
                this.setOperatorPerspectiveForward(
                    if (allianceColor == Alliance.Red) {
                        redAlliancePerspectiveRotation
                    } else {
                        blueAlliancePerspectiveRotation
                    },
                )
                hasAppliedOperatorPerspective = true
            }
        }
    }
    
    private val m_slipSysIdRoutine: SysIdRoutine = SysIdRoutine(
        SysIdRoutine.Config(
            Volts.of(0.25).per(Seconds.of(1.0)),
            null,
            null,
            ModifiedSignalLogger.logState(),
        ),
        SysIdRoutine.Mechanism(
            { volts: Measure<Voltage?> ->
                setControl(
                    driveVoltageRequest.withVoltage(
                        volts.`in`(
                            Volts,
                        ),
                    ),
                )
            },
            null,
            this,
        ),
    )
    
    fun runDriveQuasiTest(direction: SysIdRoutine.Direction?): Command {
        return m_driveSysIdRoutine.quasistatic(direction)
    }
    
    fun runDriveDynamTest(direction: SysIdRoutine.Direction?): Command {
        return m_driveSysIdRoutine.dynamic(direction)
    }
    
    fun runSteerQuasiTest(direction: SysIdRoutine.Direction?): Command {
        return m_steerSysIdRoutine.quasistatic(direction)
    }
    
    fun runSteerDynamTest(direction: SysIdRoutine.Direction?): Command {
        return m_steerSysIdRoutine.dynamic(direction)
    }
    
    fun runDriveSlipTest(): Command {
        return m_slipSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)
    }
    
    companion object {
        private const val SIM_LOOP_PERIOD = 0.005 // 5 ms
    }
}
