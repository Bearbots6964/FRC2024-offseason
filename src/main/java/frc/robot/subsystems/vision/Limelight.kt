package frc.robot.subsystems.vision

import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.*
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.CommandSwerveDrivetrain
import frc.robot.Util.RectanglePoseArea
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import kotlin.jvm.optionals.getOrNull

class Limelight(var drivetrain: CommandSwerveDrivetrain) : SubsystemBase() {
    var alliance: Alliance? = null
    private val leftCam = PhotonCamera("Left Camera")
    private val rightCam = PhotonCamera("Right Camera")
    private val noteCam = PhotonCamera("ShitCam")
    private val botToLeftCamera = Transform3d(Translation3d(0.0, 0.0, 0.0), Rotation3d(0.0, 0.0, 0.0)) // TODO get this value
    private val botToRightCamera = Transform3d(Translation3d(0.0, 0.0, 0.0), Rotation3d(0.0, 0.0, 0.0)) // TODO get this value
    private var enable = false
    private var trust = false
    private var fieldError = 0
    private var distanceError = 0
    private var botpose: Pose2d? = null
    private var aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField()
    private var rightPhotonPoseEstimator = PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, rightCam, botToRightCamera)
    private var leftPhotonPoseEstimator = PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, leftCam, botToLeftCamera)

    /** Creates a new Limelight.  */
    init {
        SmartDashboard.putNumber("Field Error", fieldError.toDouble())
        SmartDashboard.putNumber("Limelight Error", distanceError.toDouble())

    }


    fun updateLeft(): EstimatedRobotPose? {
        return leftPhotonPoseEstimator.update().getOrNull()
    }
    fun updateRight(): EstimatedRobotPose? {
        return rightPhotonPoseEstimator.update().getOrNull()
    }


    fun useLimelight(enable: Boolean) {
        this.enable = enable
    }

    fun trustLL(trust: Boolean) {
        this.trust = trust
    }

    override fun periodic() {
        val right = updateRight()
        val left = updateLeft()
        if (right != null) {
            drivetrain.addVisionMeasurement(right.estimatedPose.toPose2d(), right.timestampSeconds)
        }
        if (left != null) {
            drivetrain.addVisionMeasurement(left.estimatedPose.toPose2d(), left.timestampSeconds)
        }
    }

    fun getNoteCamAngle(): Double {
        return noteCam.getLatestResult().getBestTarget().yaw
    }

    companion object {
        private val field = RectanglePoseArea(Translation2d(0.0, 0.0), Translation2d(16.54, 8.02))
    }
}