package frc.team449.control.obstacleAvoidance

import com.pathplanner.lib.PathConstraints
import com.pathplanner.lib.PathPlanner
import com.pathplanner.lib.PathPlannerTrajectory
import com.pathplanner.lib.PathPoint
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.control.auto.HolonomicFollower
import frc.team449.control.holonomic.SwerveDrive
import frc.team449.robot2023.auto.AutoUtil
import frc.team449.robot2023.constants.RobotConstants
import frc.team449.robot2023.constants.field.FieldConstants
import kotlin.math.*

class PPAStar(
  private val drive: SwerveDrive,
  private val constraints: PathConstraints = PathConstraints(4.0, 5.25),
  private val finalPosition: Pose2d,
  private val obstacles: List<Obstacle> = FieldConstants.obstacles,
  private val aStarMap: VisGraph = MapCreator().createGraph(VisGraph(), FieldConstants.obstacles),
  private val field: Field2d
) {

  private var pathDrivingCommand: Command? = null
  private var startPoint: Node? = null

  private val finalNode = Node(finalPosition)

  init {
    aStarMap.addNode(finalNode)
    for (i in 0 until aStarMap.nodeSize) {
      val endNode = aStarMap.getNode(i)
      aStarMap.addEdge(Edge(finalNode, endNode), obstacles)
    }
  }

  fun createCommand(): Command? {
    /*
     * Add starting position to map
     */

    // Create the starting position
    startPoint = if (RobotConstants.ALLIANCE_COLOR == Alliance.Blue) {
      Node(drive.pose)
    } else {
      val blueStart = Pose2d(
        FieldConstants.fieldLength - drive.pose.x,
        drive.pose.y,
        drive.heading.plus(Rotation2d(PI))
      )
      Node(blueStart)
    }

    // What will be our trajectory for path planner to follow
    var trajectory: PathPlannerTrajectory?

    // Adds start point (current position on the field)
    startPoint?.let { aStarMap.addNode(it) }

    // Connects our starting point to all other nodes on the field (this does check
    // for obstacles)
    for (i in 0 until aStarMap.nodeSize) {
      val endNode = aStarMap.getNode(i)
      aStarMap.addEdge(Edge(startPoint!!, endNode), obstacles)
    }

    // Finds the path using A*
    val fullPath: List<Node?> = aStarMap.findPath(startPoint, finalNode) ?: return null

    // Gets speed of robot
    val chassisSpeeds = drive.currentSpeeds
    val fieldSpeeds = Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
      .rotateBy(drive.heading)
    val robotSpeeds = ChassisSpeeds(fieldSpeeds.x, fieldSpeeds.y, chassisSpeeds.omegaRadiansPerSecond)
    val startingSpeed = if (RobotConstants.ALLIANCE_COLOR == Alliance.Red) {
      max(
        0.0,
        -fieldSpeeds
          .rotateBy(
            finalPosition.translation
              .minus(drive.pose.translation)
              .angle
              .unaryMinus()
          )
          .x
      )
    } else {
      min(
        0.0,
        -fieldSpeeds
          .rotateBy(
            finalPosition.translation
              .minus(drive.pose.translation)
              .angle
              .unaryMinus()
          )
          .x
      )
    }

    // Gets heading that the robot needs to go to reach first point
    var heading = Rotation2d(
      fullPath[1]!!.x -
        startPoint!!.x,
      fullPath[1]!!.y - startPoint!!.y
    )
    val fullPathPoints = ArrayList<PathPoint>()

    /*
     * Takes our path found above and sends it to an ArrayList of Path Planner
     * Points. Also adds in midpoints
     * I would recommend commenting out addMidPoints at first when first testing
     * code
     */

    // Find path between points
    for (i in fullPath.indices) {
      if (i == 0) {
        fullPathPoints.add(
          PathPoint(
            Translation2d(startPoint!!.x, startPoint!!.y),
            heading,
            finalNode.holRot,
            startingSpeed
          )
        )
        addMidPoints(fullPathPoints, fullPath, i)
      } else if (i + 1 == fullPath.size) {
        heading = Rotation2d(
          fullPath[i]!!.x - fullPath[i - 1]!!.x,
          fullPath[i]!!.y - fullPath[i - 1]!!.y
        )
        fullPathPoints.add(
          PathPoint(
            Translation2d(finalNode.x, finalNode.y),
            heading,
            finalNode.holRot
          )
        )
      } else {
        heading = Rotation2d(
          fullPath[i + 1]!!.x - fullPath[i]!!.x,
          fullPath[i + 1]!!.y - fullPath[i]!!.y
        )
        fullPathPoints.add(
          PathPoint(
            Translation2d(fullPath[i]!!.x, fullPath[i]!!.y),
            heading,
            finalNode.holRot
          )
        )
        addMidPoints(fullPathPoints, fullPath, i)
      }
    }

    trajectory = PathPlanner.generatePath(constraints, fullPathPoints)
    // Display Trajectory
    // Change trajectory based on alliance color
    trajectory = AutoUtil.transformForAlliance(
      mutableListOf(trajectory)
    ) { RobotConstants.ALLIANCE_COLOR == Alliance.Red }[0]
    pathDrivingCommand = HolonomicFollower(
      drive,
      trajectory
    )

    field.getObject("PPAStar Path").setTrajectory(trajectory)

    return pathDrivingCommand
  }

  // Adds MidPoints so that Bézier curve doesn't curve into obstacle
  private fun addMidPoints(fullPathPoints: ArrayList<PathPoint>, fullPath: List<Node?>, i: Int) {
    val distance = hypot(
      fullPath[i + 1]!!.x - fullPath[i]!!.x,
      fullPath[i + 1]!!.y - fullPath[i]!!.y
    )

    // Adjust distance / x to have more or less midpoints lower the x the more midpoints
    val midpoints = floor(distance / 1.0).toInt()
    val heading = Rotation2d(
      fullPath[i + 1]!!.x - fullPath[i]!!.x,
      fullPath[i + 1]!!.y - fullPath[i]!!.y
    )
    for (j in 0 until midpoints) {
      fullPathPoints.add(
        PathPoint(
          Translation2d(
            fullPath[i]!!.x +
              (fullPath[i + 1]!!.x - fullPath[i]!!.x) * ((j + 1.0) / (midpoints + 1.0)),
            (
              fullPath[i]!!.y +
                (fullPath[i + 1]!!.y - fullPath[i]!!.y) * ((j + 1.0) / (midpoints + 1.0))
              )
          ),
          heading,
          finalNode.holRot
        )
      )
    }
  }
}
