package frc.team449.robot2023.commands.autoScore

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.*
import frc.team449.control.obstacleAvoidance.PPAStar
import frc.team449.robot2023.Robot
import frc.team449.robot2023.constants.field.FieldConstants
import frc.team449.robot2023.constants.subsystem.ArmConstants
import frc.team449.robot2023.subsystems.arm.control.ArmState
import kotlin.math.abs

class ScoringCommands(
  private val robot: Robot
) {

  private val levelsToArm: Map<Levels, ArmState> = mapOf(
    Levels.LOW to ArmConstants.LOW,
    Levels.MID to ArmConstants.MID,
    Levels.HIGH to ArmConstants.HIGH
  )

  private val levelsToOffsets: Map<Levels, Double> = mapOf(
    Levels.LOW to 0.0,
    Levels.MID to abs(robot.arm.kinematics.toCartesian(ArmConstants.MID).x) -
      ArmConstants.backToArmBase - FieldConstants.midNodeFromEdge,
    Levels.HIGH to abs(robot.arm.kinematics.toCartesian(ArmConstants.HIGH).x) -
      ArmConstants.backToArmBase - FieldConstants.highNodeFromEdge
  )
  fun generateCommand(position: FieldConstants.TargetPosition, level: Levels): Command {
    val finalPos = Pose2d(
      FieldConstants.PlacementPositions[position]!!.x + levelsToOffsets[level]!!,
      FieldConstants.PlacementPositions[position]!!.y,
      FieldConstants.PlacementPositions[position]!!.rotation
    )

    return ParallelCommandGroup(
      PPAStar(
        drive = robot.drive,
        finalPosition = finalPos,
        field = robot.field
      ).createCommand() ?: PrintCommand("COULD NOT GENERATE ON THE FLY TRAJECTORY!!!"),
      moveArmNearNode(robot, finalPos, levelsToArm[level] ?: ArmConstants.BACK)
    )
  }

  enum class Levels {
    LOW,
    MID,
    HIGH
  }
}
