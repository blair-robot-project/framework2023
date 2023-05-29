package frc.team449.robot2023.auto

import com.pathplanner.lib.PathPlanner
import com.pathplanner.lib.PathPlannerTrajectory

object Paths {
  object POS1 {
    /** Description of path: Just an example at position 1  */
    val EXAMPLE: MutableList<PathPlannerTrajectory> =
      PathPlanner.loadPathGroup(
        "example1",
        PathPlanner.getConstraintsFromPath("example1")
      )
  }

  object POS2 {
    val EXAMPLE: MutableList<PathPlannerTrajectory> =
      AutoUtil.transformForPosition2(
        PathPlanner.loadPathGroup(
          "example1",
          PathPlanner.getConstraintsFromPath("example1")
        )
      )
  }
}
