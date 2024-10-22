package frc.robot.subsystems.vision

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.photonvision.PhotonCamera
import org.photonvision.targeting.PhotonTrackedTarget

class VisionSubsystem : SubsystemBase() {
    private var camera: PhotonCamera = PhotonCamera("ShitCam")
    private lateinit var rot: Rotation2d
    private var countWithoutMeasurement: Int = 0

    override fun periodic() {
        // This method will be called once per scheduler run
        var result = camera.getLatestResult()
        if (result.hasTargets()) {
            var target: PhotonTrackedTarget = result.getBestTarget()
            rot = Rotation2d.fromDegrees(target.yaw)
            countWithoutMeasurement = 0
        } else {
            countWithoutMeasurement++
        }

        if (countWithoutMeasurement > 25) { // method called every 20 ms,
            // so 25 * 20 = 500 ms without a target before setting rotation to 0
            rot = Rotation2d.fromDegrees(0.0)
        }
    }

    // returns angle to nearest note relative to the front of the robot
    fun getNearestRotation(): Rotation2d {
        return rot
    }
}
