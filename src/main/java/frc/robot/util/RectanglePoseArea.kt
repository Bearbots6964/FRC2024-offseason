package frc.robot.util

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d

class RectanglePoseArea
/**
 * Create a 2D rectangular area for pose calculations.
 *
 * @param bottomLeft bottom left corner of the rectangle.
 * @param topRight top right corner of the rectangle.
 */(private val bottomLeft: Translation2d, private val topRight: Translation2d) {
    val minX: Double
        get() {
            return bottomLeft.x
        }
    
    val maxX: Double
        get() {
            return topRight.x
        }
    
    val minY: Double
        get() {
            return bottomLeft.y
        }
    
    val maxY: Double
        get() {
            return topRight.y
        }
    
    fun isPoseWithinArea(pose: Pose2d): Boolean {
        return pose.x >= bottomLeft.x
            && pose.x <= topRight.x
            && pose.y >= bottomLeft.y
            && pose.y <= topRight.y
    }
}