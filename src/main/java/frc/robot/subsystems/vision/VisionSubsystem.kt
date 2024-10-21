package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.math.geometry.Rotation2d
import org.photonvision.PhotonCamera
import org.photonvision.PhotonUtils
import org.photonvision.targeting.PhotonTrackedTarget

public class VisionSubsystem : SubsystemBase(){
    private var camera : PhotonCamera = PhotonCamera("camera1")

    public VisionSubsystem()

    //returns angle to nearest note relative to the front of the robot
    public fun getNearestRotation() : Rotation2d{
        var result = camera.getLatestResult()
        if(result.hasTargets()){
            var target : PhotonTrackedTarget = result.getBestTarget()
            return Rotation2d.fromDegrees(target.getYaw())
        }
        return Rotation2d.fromDegrees(0.0)
    }


}