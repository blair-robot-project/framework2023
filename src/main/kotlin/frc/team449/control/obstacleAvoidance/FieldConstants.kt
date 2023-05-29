package frc.team449.control.obstacleAvoidance

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.util.Units

object FieldConstants {
  val FIELD_LENGTH_METERS = Units.inchesToMeters(651.25)
  val FIELD_WIDTH_METERS = Units.inchesToMeters(315.5)
  val tapeWidth = Units.inchesToMeters(2.0)
  var standardObstacles = listOf( // Charging Station
    Obstacle(
      doubleArrayOf(
        2.40,
        5.40,
        5.40,
        2.40
      ),
      doubleArrayOf(
        4.50,
        4.50,
        1.00,
        1.00
      )
    ),
    Obstacle(
      doubleArrayOf(
        3.8,
        3.8,
        1.26
      ),
      doubleArrayOf(
        5.5,
        5.0,
        5.25
      )
    )
  )
  var shortObstacles = listOf( // Charging Station
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

  // Forces robot to go over cable. In case of defense.
  var cablePath = listOf( // Charging Station
    Obstacle(
      doubleArrayOf(
        2.48,
        5.36,
        5.36,
        2.48
      ),
      doubleArrayOf(
        4.81,
        4.81,
        1.07,
        1.07
      )
    ),
    Obstacle(
      doubleArrayOf(
        3.84,
        3.84,
        1.26
      ),
      doubleArrayOf(
        6.23,
        4.80,
        5.52
      )
    )
  )
  var PlacementPositions: Map<TargetPosition, Pose2d> = java.util.Map.of(
    TargetPosition.Position1,
    Pose2d(2.0, Grids.lowTranslations[0]!!.y, Rotation2d.fromDegrees(180.0)),
    TargetPosition.Position2,
    Pose2d(2.0, Grids.lowTranslations[1]!!.y, Rotation2d.fromDegrees(180.0)),
    TargetPosition.Position3,
    Pose2d(2.0, Grids.lowTranslations[2]!!.y, Rotation2d.fromDegrees(180.0)),
    TargetPosition.Position4,
    Pose2d(2.0, Grids.lowTranslations[3]!!.y, Rotation2d.fromDegrees(180.0)),
    TargetPosition.Position5,
    Pose2d(2.0, Grids.lowTranslations[4]!!.y, Rotation2d.fromDegrees(180.0)),
    TargetPosition.Position6,
    Pose2d(2.0, Grids.lowTranslations[5]!!.y, Rotation2d.fromDegrees(180.0)),
    TargetPosition.Position7,
    Pose2d(2.0, Grids.lowTranslations[6]!!.y, Rotation2d.fromDegrees(180.0)),
    TargetPosition.Position8,
    Pose2d(2.0, Grids.lowTranslations[7]!!.y, Rotation2d.fromDegrees(180.0)),
    TargetPosition.Position9,
    Pose2d(2.0, Grids.lowTranslations[8]!!.y, Rotation2d.fromDegrees(180.0))
  )

  // Dimensions for grids and nodes
  object Grids {
    // X layout
    val outerX = Units.inchesToMeters(54.25)
    val lowX = outerX - Units.inchesToMeters(14.25) / 2.0 // Centered when under cube

    // nodes
    val midX = outerX - Units.inchesToMeters(22.75)
    val highX = outerX - Units.inchesToMeters(39.75)

    // Y layout
    const val nodeRowCount = 9
    val nodeFirstY = Units.inchesToMeters(20.19)
    val nodeSeparationY = Units.inchesToMeters(22.0)

    // Z layout
    val cubeEdgeHigh = Units.inchesToMeters(3.0)
    val highCubeZ = Units.inchesToMeters(35.5) - cubeEdgeHigh
    val midCubeZ = Units.inchesToMeters(23.5) - cubeEdgeHigh
    val highConeZ = Units.inchesToMeters(46.0)
    val midConeZ = Units.inchesToMeters(34.0)

    // Translations (all nodes in the same column/row have the same X/Y coordinate)
    val lowTranslations = arrayOfNulls<Translation2d>(nodeRowCount)
    val midTranslations = arrayOfNulls<Translation2d>(nodeRowCount)
    val mid3dTranslations = arrayOfNulls<Translation3d>(nodeRowCount)
    val highTranslations = arrayOfNulls<Translation2d>(nodeRowCount)
    val high3dTranslations = arrayOfNulls<Translation3d>(nodeRowCount)

    init {
      for (i in 0 until nodeRowCount) {
        val isCube = i == 1 || i == 4 || i == 7
        lowTranslations[i] = Translation2d(lowX, nodeFirstY + nodeSeparationY * i)
        midTranslations[i] = Translation2d(midX, nodeFirstY + nodeSeparationY * i)
        mid3dTranslations[i] = Translation3d(
          midX,
          nodeFirstY + nodeSeparationY * i,
          if (isCube) midCubeZ else midConeZ
        )
        high3dTranslations[i] = Translation3d(
          highX,
          nodeFirstY + nodeSeparationY * i,
          if (isCube) highCubeZ else highConeZ
        )
        highTranslations[i] = Translation2d(highX, nodeFirstY + nodeSeparationY * i)
      }
    }

    // Complex low layout (shifted to account for cube vs cone rows and wide edge
    // nodes)
    val complexLowXCones = outerX - Units.inchesToMeters(16.0) / 2.0 // Centered X under

    // cone nodes
    val complexLowXCubes = lowX // Centered X under cube nodes
    val complexLowOuterYOffset = nodeFirstY - Units.inchesToMeters(3.0) - Units.inchesToMeters(25.75) / 2.0
    val complexLowTranslations = arrayOf(
      Translation2d(complexLowXCones, nodeFirstY - complexLowOuterYOffset),
      Translation2d(complexLowXCubes, nodeFirstY + nodeSeparationY * 1),
      Translation2d(complexLowXCones, nodeFirstY + nodeSeparationY * 2),
      Translation2d(complexLowXCones, nodeFirstY + nodeSeparationY * 3),
      Translation2d(complexLowXCubes, nodeFirstY + nodeSeparationY * 4),
      Translation2d(complexLowXCones, nodeFirstY + nodeSeparationY * 5),
      Translation2d(complexLowXCones, nodeFirstY + nodeSeparationY * 6),
      Translation2d(complexLowXCubes, nodeFirstY + nodeSeparationY * 7),
      Translation2d(
        complexLowXCones,
        nodeFirstY + nodeSeparationY * 8 + complexLowOuterYOffset
      )
    )
  }

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
