package frc.robot

import com.ctre.phoenix6.SignalLogger
import com.ctre.phoenix6.Utils
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain
import com.pathplanner.lib.util.PathPlannerLogging
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.networktables.DoubleArrayPublisher
import edu.wpi.first.networktables.DoublePublisher
import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.StringPublisher
import edu.wpi.first.networktables.StructArrayPublisher
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj.util.Color8Bit
import org.littletonrobotics.junction.AutoLogOutput

class Telemetry
/**
 * Construct a telemetry object, with the specified max speed
 * of the robot.
 *
 * @param maxSpeed Maximum speed in meters per second.
 */(private val maxSpeed: Double) {
    private val publisher: StructArrayPublisher<SwerveModuleState> =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish()
    /* What to publish over networktables for telemetry */
    private val inst: NetworkTableInstance = NetworkTableInstance.getDefault()
    
    /** Robot pose for field positioning. */
    private val table: NetworkTable = inst.getTable("Pose")
    private val fieldPub: DoubleArrayPublisher = table.getDoubleArrayTopic("robotPose").publish()
    private val trajPub: DoubleArrayPublisher = table.getDoubleArrayTopic("traj").publish()
    private val fieldTypePub: StringPublisher = table.getStringTopic(".type").publish()
    
    // Robot speeds for general checking
    
    private val driveStats: NetworkTable = inst.getTable("Drive")
    private val velocityX: DoublePublisher = driveStats.getDoubleTopic("Velocity X").publish()
    private val velocityY: DoublePublisher = driveStats.getDoubleTopic("Velocity Y").publish()
    private val speed: DoublePublisher = driveStats.getDoubleTopic("Speed").publish()
    private val odomFreq: DoublePublisher =
        driveStats.getDoubleTopic("Odometry Frequency").publish()
    
    /* Keep a reference of the last pose to calculate the speeds */
    @AutoLogOutput
    private var lastPose = Pose2d()
    private var lastTime = Utils.getCurrentTimeSeconds()
    
    /* Mechanisms to represent the swerve module states */
    private val moduleMechanisms = arrayOf(
        Mechanism2d(1.0, 1.0),
        Mechanism2d(1.0, 1.0),
        Mechanism2d(1.0, 1.0),
        Mechanism2d(1.0, 1.0),
    )
    
    /* A direction and length changing ligament for speed representation */
    private val moduleSpeeds = arrayOf(
        moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5)
            .append(MechanismLigament2d("Speed", 0.5, 0.0)),
        moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5)
            .append(MechanismLigament2d("Speed", 0.5, 0.0)),
        moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5)
            .append(MechanismLigament2d("Speed", 0.5, 0.0)),
        moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5)
            .append(MechanismLigament2d("Speed", 0.5, 0.0)),
    )
    
    /* A direction changing and length constant ligament for module direction */
    private val moduleDirections = arrayOf(
        moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
            .append(MechanismLigament2d("Direction", 0.1, 0.0, 0.0, Color8Bit(Color.kWhite))),
        moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
            .append(MechanismLigament2d("Direction", 0.1, 0.0, 0.0, Color8Bit(Color.kWhite))),
        moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
            .append(MechanismLigament2d("Direction", 0.1, 0.0, 0.0, Color8Bit(Color.kWhite))),
        moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
            .append(MechanismLigament2d("Direction", 0.1, 0.0, 0.0, Color8Bit(Color.kWhite))),
    )
    
    init {
        SignalLogger.start()
        
        PathPlannerLogging.setLogActivePathCallback { poses: List<Pose2d> ->
            val arr = DoubleArray(poses.size * 3)
            var ndx = 0
            for (pose in poses) {
                arr[ndx + 0] = pose.x
                arr[ndx + 1] = pose.y
                arr[ndx + 2] = pose.rotation.degrees
                ndx += 3
            }
            trajPub.set(arr)
        }
        
        SmartDashboard.putData("Swerve Drive") { builder ->
            builder.setSmartDashboardType("SwerveDrive")
            builder.addDoubleProperty("Front Left Angle", { moduleDirections[0].angle }, null)
            builder.addDoubleProperty("Front Left Velocity", { moduleSpeeds[0].length }, null)
            
            builder.addDoubleProperty("Front Right Angle", { moduleDirections[1].angle }, null)
            builder.addDoubleProperty("Front Right Velocity", { moduleSpeeds[1].length }, null)
            
            builder.addDoubleProperty("Back Left Angle", { moduleDirections[2].angle }, null)
            builder.addDoubleProperty("Back Left Velocity", { moduleSpeeds[2].length }, null)
            
            builder.addDoubleProperty("Back Right Angle", { moduleDirections[3].angle }, null)
            builder.addDoubleProperty("Back Right Velocity", { moduleSpeeds[3].length }, null)
            builder.addDoubleProperty("Robot Angle", { lastPose.rotation.radians }, null)
        }
        
    }
    
    /* Accept the swerve drive state and telemeterize it to smartdashboard */
    fun telemeterize(state: SwerveDrivetrain.SwerveDriveState) {
        /* Telemeterize the pose */
        val pose = state.Pose
        fieldTypePub.set("Field2d")
        fieldPub.set(
            doubleArrayOf(
                pose.x,
                pose.y,
                pose.rotation.degrees
            )
        )
        
        /* Telemeterize the robot's general speeds */
        val currentTime = Utils.getCurrentTimeSeconds()
        val diffTime = currentTime - lastTime
        lastTime = currentTime
        val distanceDiff = pose.minus(lastPose).translation
        lastPose = pose
        
        val velocities = distanceDiff.div(diffTime)
        
        speed.set(velocities.norm)
        velocityX.set(velocities.x)
        velocityY.set(velocities.y)
        odomFreq.set(1.0 / state.OdometryPeriod)
        
        /* Telemeterize the module's states */
        for (i in 0..3) {
            moduleSpeeds[i].setAngle(state.ModuleStates[i].angle)
            moduleDirections[i].setAngle(state.ModuleStates[i].angle)
            moduleSpeeds[i].length = state.ModuleStates[i].speedMetersPerSecond / (2 * maxSpeed)
            
            SmartDashboard.putData("Module $i", moduleMechanisms[i])
        }
        
        // Periodically send a set of module states
        publisher.set(state.ModuleStates)
        
        SignalLogger.writeDoubleArray(
            "odometry",
            doubleArrayOf(pose.x, pose.y, pose.rotation.degrees)
        )
        SignalLogger.writeDouble("odom period", state.OdometryPeriod, "seconds")
    }
}
