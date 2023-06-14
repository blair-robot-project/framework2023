package frc.team449.robot2023.auto.routines

import com.pathplanner.lib.PathPlannerTrajectory
import frc.team449.control.auto.HolonomicRoutine
import frc.team449.control.auto.RoutineStructure
import frc.team449.robot2023.Robot
import frc.team449.robot2023.auto.AutoUtil
import frc.team449.robot2023.auto.Paths
import frc.team449.robot2023.auto.PositionChooser

class EdgeCube(
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
        "stopIntake" to AutoUtil.retractAndStow(robot)
      )
    )

  override val trajectory: MutableList<PathPlannerTrajectory> =
    if (position == PositionChooser.Positions.FARCUBE) {
      if (isRed) AutoUtil.transformForAlliance(Paths.FAR.CUBE) { true } else Paths.FAR.CUBE
    } else {
      if (isRed) AutoUtil.transformForAlliance(Paths.WALL.CUBE) { true } else Paths.WALL.CUBE
    }
}
