package frc.robot.subsystems.vision

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.*
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Util.RectanglePoseArea
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.targeting.PhotonPipelineResult
import org.photonvision.targeting.PhotonTrackedTarget
import java.util.*
import kotlin.jvm.optionals.getOrNull
import kotlin.math.abs

object VisionSubsystem : SubsystemBase() {
    var alliance: Alliance? = null
    private val leftCam = PhotonCamera("Left Camera")
    private val rightCam = PhotonCamera("Right Camera")
    private val noteCam = PhotonCamera("ShitCam")
    private val botToLeftCamera = Transform3d(
        Translation3d(-2.25, 11.75, 7.5),
        Rotation3d(Units.Degree.of(10.0).`in`(Units.Radian), 0.0, 0.0),
    ) // TODO get this value
    private val botToRightCamera = Transform3d(
        Translation3d(2.25, -11.75, 7.5),
        Rotation3d(Units.Degree.of(-10.0).`in`(Units.Radian), 0.0, 0.0),
    ) // TODO get this value
    private var enable = false
    private var trust = false
    private var fieldError = 0
    private var distanceError = 0
    private var botpose: Pose2d? = null
    private var aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo)
    private var rightPhotonPoseEstimator = PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, rightCam, botToRightCamera)

    private var leftPhotonPoseEstimator = PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, leftCam, botToLeftCamera)
    private val isSim = RobotBase.isSimulation()

    /** Creates a new Limelight.  */
    init {
        rightPhotonPoseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY)
        leftPhotonPoseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY)

        SmartDashboard.putNumber("Field Error", fieldError.toDouble())
        SmartDashboard.putNumber("Limelight Error", distanceError.toDouble())
    }

    /**
     * Calculates standard deviations for the right camera based on the number of targets detected. Heuristic.
     */
    private fun updateLeftEstimationStdDevs(
        visionEst: Optional<EstimatedRobotPose>,
        targets: List<PhotonTrackedTarget>,
    ) {
    }

    // singleton class
    val instance: VisionSubsystem = VisionSubsystem

    fun useLimelight(enable: Boolean) {
        this.enable = enable
    }

    fun trustLL(trust: Boolean) {
        this.trust = trust
    }

    override fun periodic() {
    }

    fun getNoteCamYaw(): Double {
        try {
            if (!isSim) {
                return if (noteCam.getLatestResult().getBestTarget() != null) {
                    noteCam.getLatestResult()
                        .getBestTarget().yaw
                } else {
                    0.0
                }
            } else {
                return 0.0
            }
        } finally {
        }
    }

    fun getNoteCamPitch(): Double {
        try {
            if (!isSim) {
                return if (noteCam.getLatestResult().getBestTarget() != null) {
                    noteCam.getLatestResult()
                        .getBestTarget().pitch
                } else {
                    0.0
                }
            } else {
                return 0.0
            }
        } finally {
        }
    }

    fun isTarget(): Boolean {
        try {
            return noteCam.latestResult.hasTargets()
        } finally {
            false
        }
    }

    fun isCloseEnoughToDone(): Boolean {
        try {
            return abs(noteCam.latestResult.bestTarget.yaw) < 5 && abs(noteCam.latestResult.bestTarget.pitch) < 5
        } finally {
            false
        }
    }

    private val field = RectanglePoseArea(Translation2d(0.0, 0.0), Translation2d(16.54, 8.02))

    @JvmStatic
    fun updateLeft(): EstimatedRobotPose? {
        return VisionSubsystem.instance.leftPhotonPoseEstimator.update().getOrNull()
    }

    @JvmStatic
    fun updateRight(): EstimatedRobotPose? {
        return VisionSubsystem.instance.rightPhotonPoseEstimator.update().getOrNull()
    }

    fun getLeftResult(): PhotonPipelineResult? {
        return leftCam.latestResult
    }

    fun getRightResult(): PhotonPipelineResult? {
        return rightCam.latestResult
    }
}
