package frc.team449.robot2023.commands.autoScore

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team449.robot2023.Robot
import frc.team449.robot2023.constants.RobotConstants
import frc.team449.robot2023.constants.field.FieldConstants
import frc.team449.robot2023.constants.subsystem.ArmConstants
import frc.team449.robot2023.subsystems.arm.control.ArmFollower
import frc.team449.robot2023.subsystems.arm.control.ArmState
import kotlin.math.PI
import kotlin.math.abs

class moveArmNearNode(
  private val robot: Robot,
  private val finalPos: Pose2d,
  private val armState: ArmState
) : CommandBase() {

  private val armCommand = ArmFollower(robot.arm) { robot.arm.chooseTraj(armState) }
  private val stowArmCommand = ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.BACK) }

  private fun nearNode(): Boolean {
    val difference = if (RobotConstants.ALLIANCE_COLOR == DriverStation.Alliance.Blue) {
      robot.drive.pose.relativeTo(finalPos)
    } else {
      robot.drive.pose.relativeTo(Pose2d(FieldConstants.fieldLength - finalPos.x, finalPos.y, finalPos.rotation - Rotation2d(PI)))
    }
    return abs(difference.x) < FieldConstants.withinNode.x &&
      abs(difference.y) < FieldConstants.withinNode.y &&
      abs(difference.rotation.radians) < FieldConstants.withinNode.rotation.radians
  }

  override fun initialize() {
    stowArmCommand.schedule()
  }

  override fun execute() {
    if (!stowArmCommand.isScheduled &&
      !armCommand.isScheduled &&
      RobotConstants.ALLIANCE_COLOR == DriverStation.Alliance.Blue &&
      nearNode()
    ) {
      armCommand.schedule()
    } else if (!stowArmCommand.isScheduled &&
      !armCommand.isScheduled &&
      RobotConstants.ALLIANCE_COLOR == DriverStation.Alliance.Red &&
      nearNode()
    ) {
      armCommand.schedule()
    }
  }

  override fun isFinished(): Boolean {
    return armCommand.isFinished && armCommand.isScheduled
  }
}
