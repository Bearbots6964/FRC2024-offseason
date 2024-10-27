package frc.robot.subsystems.drive

import com.ctre.phoenix6.SignalLogger
import com.ctre.phoenix6.Utils
import com.ctre.phoenix6.swerve.SwerveDrivetrain
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType
import com.ctre.phoenix6.swerve.SwerveModuleConstants
import com.ctre.phoenix6.swerve.SwerveRequest
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.pathplanner.lib.util.DriveFeedforwards
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism
import frc.robot.util.RectanglePoseArea
import frc.robot.subsystems.vision.VisionSubsystem
import java.util.function.DoubleSupplier
import java.util.function.Supplier
import kotlin.math.PI

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
class CommandSwerveDrivetrain : SwerveDrivetrain, Subsystem {
    private var m_simNotifier: Notifier? = null
    private var m_lastSimTime = 0.0

    private var thetaController: PIDController = PIDController(1.1, 0.0, 0.125) // TODO tune this


    /* Keep track if we've ever applied the operator perspective before or not */
    private var m_hasAppliedOperatorPerspective = false

    /* Swerve request to apply during path following */
    private val m_applyRobotSpeeds = SwerveRequest.ApplyChassisSpeeds()

    /* Swerve requests to apply during SysId characterization */
    private val m_translationCharacterization = SwerveRequest.SysIdSwerveTranslation()
    private val m_steerCharacterization = SwerveRequest.SysIdSwerveSteerGains()
    private val m_rotationCharacterization = SwerveRequest.SysIdSwerveRotation()

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private val m_sysIdRoutineTranslation = SysIdRoutine(
        SysIdRoutine.Config(
            null,  // Use default ramp rate (1 V/s)
            Units.Volts.of(4.0),  // Reduce dynamic step voltage to 4 V to prevent brownout
            null
        )  // Use default timeout (10 s)
        // Log state with SignalLogger class
        { state: SysIdRoutineLog.State -> SignalLogger.writeString("SysIdTranslation_State", state.toString()) },
        Mechanism(
            { output: Voltage? -> setControl(m_translationCharacterization.withVolts(output)) }, null, this
        )
    )

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private val m_sysIdRoutineSteer = SysIdRoutine(
        SysIdRoutine.Config(
            null,  // Use default ramp rate (1 V/s)
            Units.Volts.of(7.0),  // Use dynamic voltage of 7 V
            null
        )  // Use default timeout (10 s)
        // Log state with SignalLogger class
        { state: SysIdRoutineLog.State -> SignalLogger.writeString("SysIdSteer_State", state.toString()) }, Mechanism(
            { volts: Voltage? -> setControl(m_steerCharacterization.withVolts(volts)) }, null, this
        )
    )

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private val m_sysIdRoutineRotation = SysIdRoutine(
        SysIdRoutine.Config( /* This is in radians per second², but SysId only supports "volts per second" */
            Units.Volts.of(Math.PI / 6)
                .per(Units.Second),  /* This is in radians per second, but SysId only supports "volts" */
            Units.Volts.of(Math.PI), null
        )  // Use default timeout (10 s)
        // Log state with SignalLogger class
        { state: SysIdRoutineLog.State -> SignalLogger.writeString("SysIdRotation_State", state.toString()) },
        Mechanism(
            { output: Voltage ->
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.`in`(Units.Volts)))/* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.`in`(Units.Volts))
            }, null, this
        )
    )

    /* The SysId routine to test */
    private val m_sysIdRoutineToApply = m_sysIdRoutineTranslation

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     *
     * This constructs the underlying hardware devices, so user should not construct
     * the devices themselves. If they need the devices, they can access them
     * through getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    constructor(drivetrainConstants: SwerveDrivetrainConstants, vararg modules: SwerveModuleConstants?) : super(
        drivetrainConstants, *modules
    ) {
        if (Utils.isSimulation()) {
            startSimThread()
        }
        configureAutoBuilder()
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     *
     * This constructs the underlying hardware devices, so user should not construct
     * the devices themselves. If they need the devices, they can access them
     * through getters in the classes.
     *
     * @param drivetrainConstants        Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency    The frequency to run the odometry loop. If
     * unspecified or set to 0 Hz, this is 250 Hz on
     * CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                    Constants for each specific module
     */
    constructor(
        drivetrainConstants: SwerveDrivetrainConstants,
        OdometryUpdateFrequency: Double,
        vararg modules: SwerveModuleConstants?,
    ) : super(drivetrainConstants, OdometryUpdateFrequency, *modules) {
        if (Utils.isSimulation()) {
            startSimThread()
        }
        configureAutoBuilder()
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     *
     * This constructs the underlying hardware devices, so user should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants        Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency    The frequency to run the odometry loop. If
     * unspecified or set to 0 Hz, this is 250 Hz on
     * CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation  The standard deviation for odometry calculation
     * @param visionStandardDeviation    The standard deviation for vision calculation
     * @param modules                    Constants for each specific module
     */
    constructor(
        drivetrainConstants: SwerveDrivetrainConstants, odometryUpdateFrequency: Double,
        odometryStandardDeviation: Matrix<N3?, N1?>, visionStandardDeviation: Matrix<N3?, N1?>,
        vararg modules: SwerveModuleConstants?,
    ) : super(
        drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, *modules
    ) {
        if (Utils.isSimulation()) {
            startSimThread()
        }
        configureAutoBuilder()
    }

    private fun configureAutoBuilder() {
        try {
            val config = RobotConfig.fromGUISettings()
            AutoBuilder.configure(
                { state.Pose },  // Supplier of current robot pose
                { location: Pose2d? -> this.seedFieldRelative(location) },  // Consumer for seeding pose against auto
                { state.Speeds },  // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                { speeds: ChassisSpeeds?, feedforwards: DriveFeedforwards ->
                    setControl(
                        m_applyRobotSpeeds.withSpeeds(speeds)
                            .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                            .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                    )
                },
                PPHolonomicDriveController(
                    PIDConstants(0.5, 0.0, 0.0),  // PID constants for translation
                    PIDConstants(5.0, 0.0, 0.0) // PID constants for rotation
                ),
                config,  // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                { DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red },
                this // Subsystem for requirements
            )
        } catch (ex: Exception) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.stackTrace)
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    fun applyRequest(requestSupplier: Supplier<SwerveRequest?>): Command {
        return run { this.setControl(requestSupplier.get()) }
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by [.m_sysIdRoutineToApply].
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    fun sysIdQuasistatic(direction: SysIdRoutine.Direction?): Command {
        return m_sysIdRoutineToApply.quasistatic(direction)
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by [.m_sysIdRoutineToApply].
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    fun sysIdDynamic(direction: SysIdRoutine.Direction?): Command {
        return m_sysIdRoutineToApply.dynamic(direction)
    }

    init {
        OdometryThread().start() // shrug
    }

    val field: RectanglePoseArea = RectanglePoseArea(Translation2d(0.0, 0.0), Translation2d(16.54, 8.02))

    override fun periodic() {/*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent { allianceColor: Alliance ->
                setOperatorPerspectiveForward(
                    if (allianceColor == Alliance.Red) kRedAlliancePerspectiveRotation
                    else kBlueAlliancePerspectiveRotation
                )
                m_hasAppliedOperatorPerspective = true
            }
        }

        var left = VisionSubsystem.instance.updateLeft()
        var right = VisionSubsystem.instance.updateRight()

        if (left !== null) {
            var dist = VisionSubsystem.instance.getLeftResult()?.bestTarget?.bestCameraToTarget?.translation?.getDistance(Translation3d())
            var confidence = 1 - ((dist!! - 1) / 6) // 1 - ((dist - min) / range)
            if(field.isPoseWithinArea(left.estimatedPose.toPose2d())) {
                addVisionMeasurement(left.estimatedPose.toPose2d(), left.timestampSeconds, VecBuilder.fill(confidence, confidence, 0.01)) // [x, y, theta]
            }
        }
        if (right !== null) {
            var dist = VisionSubsystem.instance.getRightResult()?.bestTarget?.bestCameraToTarget?.translation?.getDistance(Translation3d())
            var confidence = 1 - ((dist!! - 1) / 6) // 1 - ((dist - min) / range)
            if(field.isPoseWithinArea(right.estimatedPose.toPose2d())) {
                addVisionMeasurement(right.estimatedPose.toPose2d(), right.timestampSeconds, VecBuilder.fill(confidence, confidence, 0.01)) // [x, y, theta]
            }
        }


    }

    private fun startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds()

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = Notifier {
            val currentTime = Utils.getCurrentTimeSeconds()
            val deltaTime = currentTime - m_lastSimTime
            m_lastSimTime = currentTime

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage())
        }
        m_simNotifier!!.startPeriodic(kSimLoopPeriod)
    }

    companion object {
        private const val kSimLoopPeriod = 0.005 // 5 ms

        /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
        private val kBlueAlliancePerspectiveRotation: Rotation2d = Rotation2d.fromDegrees(0.0)

        /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
        private val kRedAlliancePerspectiveRotation: Rotation2d = Rotation2d.fromDegrees(180.0)
    }

    fun rotateToAngleDiff(angle: DoubleSupplier): Command {
        return run {
            // first get the current angle
            val currentAngle = state.Pose.rotation.degrees
            // then get the difference between the current angle and the target angle
            val angleDifference =
                Units.Degree.of(angle.asDouble - currentAngle).`in`(Units.Radian) // convert to radians
            applyRequest {
                SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage).withDeadband(0.0)
                    .withRotationalDeadband(0.0).withRotationalRate(thetaController.calculate(angle.asDouble) * 2 * PI)
                    .withVelocityX(0.0).withVelocityY(0.0)
            }.ignoringDisable(true).addRequirements(this)


        }
    }
}
