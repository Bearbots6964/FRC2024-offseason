package frc.robot.util

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d

class RectanglePoseArea
/**
 * Create a 2D rectangular area for pose calculations.
 *
 * @param bottomLeft bottom left corner of the rectangle.
 * @param topRight top right corner of the rectangle.
 */(val bottomLeftPoint: Translation2d, val topRightPoint: Translation2d) {
    val minX: Double
        get() {
            return bottomLeftPoint.getX()
        }

    val maxX: Double
        get() {
            return topRightPoint.getX()
        }

    val minY: Double
        get() {
            return bottomLeftPoint.getY()
        }

    val maxY: Double
        get() {
            return topRightPoint.getY()
        }

    fun isPoseWithinArea(pose: Pose2d): Boolean {
        return pose.getX() >= bottomLeftPoint.getX() && pose.getX() <= topRightPoint.getX() && pose.getY() >= bottomLeftPoint.getY() && pose.getY() <= topRightPoint.getY()
    }
}
