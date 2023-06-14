package frc.team449.robot2023.auto.routines

import com.pathplanner.lib.PathPlannerTrajectory
import frc.team449.control.auto.HolonomicRoutine
import frc.team449.control.auto.RoutineStructure
import frc.team449.robot2023.Robot
import frc.team449.robot2023.auto.AutoUtil
import frc.team449.robot2023.auto.Paths
import frc.team449.robot2023.auto.PositionChooser
import frc.team449.robot2023.commands.autoBalance.AutoBalance

class EdgeConeStation(
  robot: Robot,
  position: PositionChooser.Positions,
  isRed: Boolean
) : RoutineStructure {

  override val routine =
    HolonomicRoutine(
      drive = robot.drive,
      eventMap = hashMapOf(
        "dropCone" to AutoUtil.stowDropCone(robot),
        "stowArm" to AutoUtil.deployCone(robot),
        "stopIntake" to AutoUtil.retractAndStow(robot),
        "balanceStation" to AutoBalance.create(robot.drive)
      )
    )

  override val trajectory: MutableList<PathPlannerTrajectory> =
    if (position == PositionChooser.Positions.FARCONE) {
      if (isRed) AutoUtil.transformForAlliance(Paths.FAR.CONESTATION) { true } else Paths.FAR.CONESTATION
    } else {
      if (isRed) AutoUtil.transformForAlliance(Paths.WALL.CONESTATION) { true } else Paths.WALL.CONESTATION
    }
}
