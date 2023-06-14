package frc.team449.robot2023.auto.routines

import com.pathplanner.lib.PathPlannerTrajectory
import frc.team449.control.auto.HolonomicRoutine
import frc.team449.control.auto.RoutineStructure
import frc.team449.robot2023.Robot
import frc.team449.robot2023.auto.AutoUtil
import frc.team449.robot2023.auto.Paths
import frc.team449.robot2023.auto.PositionChooser
import frc.team449.robot2023.commands.autoBalance.AutoBalance

class EdgeCubeStation(
  robot: Robot,
  position: PositionChooser.Positions,
  isRed: Boolean
) : RoutineStructure {

  override val routine =
    HolonomicRoutine(
      drive = robot.drive,
      eventMap = hashMapOf(
        "dropCube" to AutoUtil.stowDropCube(robot),
        "stowArm" to AutoUtil.deployCone(robot),
        "stopIntake" to AutoUtil.retractAndStow(robot),
        "balanceStation" to AutoBalance.create(robot.drive)
      )
    )

  override val trajectory: MutableList<PathPlannerTrajectory> =
    if (position == PositionChooser.Positions.FARCUBE) {
      if (isRed) AutoUtil.transformForAlliance(Paths.FAR.CUBESTATION) { true } else Paths.FAR.CUBESTATION
    } else {
      if (isRed) AutoUtil.transformForAlliance(Paths.WALL.CUBESTATION) { true } else Paths.WALL.CUBESTATION
    }
}
