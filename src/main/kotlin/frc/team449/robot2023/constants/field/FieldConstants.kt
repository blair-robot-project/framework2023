package frc.team449.robot2023.constants.field

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import frc.team449.control.obstacleAvoidance.Obstacle
import frc.team449.robot2023.constants.RobotConstants

object FieldConstants {
  const val fieldLength = 16.54
  const val fieldWidth = 8.02

  const val inCommunity = 2.5

  var obstacles = listOf( // Charging Station
    Obstacle(
      doubleArrayOf(
        2.90,
        4.90,
        4.90,
        2.90
      ),
      doubleArrayOf(
        4.00,
        4.00,
        1.50,
        1.50
      )
    ),
    Obstacle(
      doubleArrayOf(
        3.33,
        3.33,
        1.4,
        1.4
      ),
      doubleArrayOf(
        5.3,
        5.75,
        5.3,
        5.75
      )
    )
  )

  val wallNodeY = Units.inchesToMeters(20.19)
  val nodeSeparationY = Units.inchesToMeters(22.0)

  val nodeX = Units.inchesToMeters(54.05) + RobotConstants.ROBOT_LENGTH / 2

  val midNodeFromEdge = Units.inchesToMeters(22.7)
  val highNodeFromEdge = Units.inchesToMeters(39.73)

  val PlacementPositions: Map<TargetPosition, Pose2d> = mapOf(
    TargetPosition.Position1 to Pose2d(nodeX, wallNodeY, Rotation2d()),
    TargetPosition.Position2 to Pose2d(nodeX, wallNodeY + nodeSeparationY, Rotation2d()),
    TargetPosition.Position3 to Pose2d(nodeX, wallNodeY + 2 * nodeSeparationY, Rotation2d()),
    TargetPosition.Position4 to Pose2d(nodeX, wallNodeY + 3 * nodeSeparationY, Rotation2d()),
    TargetPosition.Position5 to Pose2d(nodeX, wallNodeY + 4 * nodeSeparationY, Rotation2d()),
    TargetPosition.Position6 to Pose2d(nodeX, wallNodeY + 5 * nodeSeparationY, Rotation2d()),
    TargetPosition.Position7 to Pose2d(nodeX, wallNodeY + 6 * nodeSeparationY, Rotation2d()),
    TargetPosition.Position8 to Pose2d(nodeX, wallNodeY + 7 * nodeSeparationY, Rotation2d()),
    TargetPosition.Position9 to Pose2d(nodeX, wallNodeY + 8 * nodeSeparationY, Rotation2d())
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
