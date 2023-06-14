package frc.team449.control.obstacleAvoidance

import edu.wpi.first.math.util.Units
import frc.team449.robot2023.constants.drives.SwerveConstants

object FieldConstants {
  val FIELD_LENGTH_METERS = Units.inchesToMeters(651.25)
  val FIELD_WIDTH_METERS = Units.inchesToMeters(315.5)
  val tapeWidth = Units.inchesToMeters(2.0)

  var shortObstacles = listOf( // Charging Station
    Obstacle(
      doubleArrayOf(
        2.90 - (SwerveConstants.WHEELBASE + Units.inchesToMeters(3.5)) / 2,
        4.90 + (SwerveConstants.WHEELBASE + Units.inchesToMeters(3.5)) / 2,
        4.90 + (SwerveConstants.WHEELBASE + Units.inchesToMeters(3.5)) / 2,
        2.90 - (SwerveConstants.WHEELBASE + Units.inchesToMeters(3.5)) / 2
      ),
      doubleArrayOf(
        4.00 + (SwerveConstants.TRACKWIDTH + Units.inchesToMeters(3.5)) / 2,
        4.00 + (SwerveConstants.TRACKWIDTH + Units.inchesToMeters(3.5)) / 2,
        1.50 - (SwerveConstants.TRACKWIDTH + Units.inchesToMeters(3.5)) / 2,
        1.50 - (SwerveConstants.TRACKWIDTH + Units.inchesToMeters(3.5)) / 2
      )
    ),
    Obstacle(
      doubleArrayOf(
        3.33 + (SwerveConstants.WHEELBASE + Units.inchesToMeters(3.5)) / 2,
        3.33 + (SwerveConstants.WHEELBASE + Units.inchesToMeters(3.5)) / 2,
        1.4 - (SwerveConstants.WHEELBASE + Units.inchesToMeters(3.5)) / 2,
        1.4 - (SwerveConstants.WHEELBASE + Units.inchesToMeters(3.5)) / 2
      ),
      doubleArrayOf(
        5.3 - (SwerveConstants.TRACKWIDTH + Units.inchesToMeters(3.5)) / 2,
        5.75 + (SwerveConstants.TRACKWIDTH + Units.inchesToMeters(3.5)) / 2,
        5.3 - (SwerveConstants.TRACKWIDTH + Units.inchesToMeters(3.5)) / 2,
        5.75 + (SwerveConstants.TRACKWIDTH + Units.inchesToMeters(3.5)) / 2
      )
    )
  )

  enum class TargetPosition {
    Position1,
    Position2,
    Position3,
    Position4,
    Position5,
    Position6,
    Position7,
    Position8,
    Position9
  }
}
