package frc.team449.robot2023.auto.routines

import com.pathplanner.lib.PathPlannerTrajectory
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.PrintCommand
import frc.team449.control.auto.HolonomicRoutine
import frc.team449.control.auto.RoutineStructure
import frc.team449.robot2023.Robot
import frc.team449.robot2023.auto.AutoUtil
import frc.team449.robot2023.auto.Paths
import frc.team449.robot2023.commands.autoBalance.AutoBalance
import frc.team449.robot2023.constants.vision.VisionConstants
import frc.team449.robot2023.subsystems.arm.ArmPaths
import frc.team449.robot2023.subsystems.arm.control.ArmFollower

class CenterCubeStation(
  robot: Robot,
  isRed: Boolean
) : RoutineStructure {

  override val routine =
    HolonomicRoutine(
      drive = robot.drive,
      eventMap = hashMapOf(
        "dropCube" to InstantCommand({
          VisionConstants.MAX_DISTANCE_SINGLE_TAG = 0.0
          VisionConstants.MAX_DISTANCE_MULTI_TAG = 0.0
        }).andThen(AutoUtil.stowDropCube(robot)),
        "stowArm" to ArmFollower(robot.arm) { ArmPaths.highStow },
        "balanceStation" to AutoBalance.create(robot.drive).alongWith(PrintCommand("Running the auto balance"))
      ),
      timeout = 0.0
    )

  override val trajectory: MutableList<PathPlannerTrajectory> =
    if (isRed) AutoUtil.transformForAlliance(Paths.CENTER.CUBEBALANCE) { true } else Paths.CENTER.CUBEBALANCE
}
