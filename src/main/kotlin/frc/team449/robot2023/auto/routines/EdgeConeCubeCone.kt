package frc.team449.robot2023.auto.routines

import com.pathplanner.lib.PathPlannerTrajectory
import frc.team449.control.auto.HolonomicRoutine
import frc.team449.control.auto.RoutineStructure
import frc.team449.robot2023.Robot
import frc.team449.robot2023.auto.AutoUtil
import frc.team449.robot2023.auto.Paths
import frc.team449.robot2023.auto.PositionChooser
import frc.team449.robot2023.subsystems.arm.ArmPaths
import frc.team449.robot2023.subsystems.arm.control.ArmFollower

class EdgeConeCubeCone(
  robot: Robot,
  position: PositionChooser.Positions,
  isRed: Boolean
) : RoutineStructure {

  override val routine =
    HolonomicRoutine(
      drive = robot.drive,
      eventMap = hashMapOf(
        "dropCube" to AutoUtil.dropCube(robot),
        "stowArm" to AutoUtil.deployCube(robot),
        "highArm" to ArmFollower(robot.arm) { ArmPaths.cubeHigh },
        "stowCube" to AutoUtil.deployCube(robot),
        "midCube" to ArmFollower(robot.arm) { ArmPaths.coneHigh },
        "stopIntake" to AutoUtil.retractGroundIntake(robot),
        "dropCone" to AutoUtil.stowDropCone(robot),
        "stopCubeIntake" to AutoUtil.retractGroundIntake(robot),
        "dropCone2" to AutoUtil.dropCone(robot)
      ),
      timeout = 0.5
    )

  override val trajectory: MutableList<PathPlannerTrajectory> =
    if (position == PositionChooser.Positions.FARCONE) {
      if (isRed) AutoUtil.transformForAlliance(Paths.FAR.CONECUBECONE) { true } else Paths.FAR.CONECUBECONE
    } else {
      if (isRed) AutoUtil.transformForAlliance(Paths.WALL.CONECUBECONE) { true } else Paths.WALL.CONECUBECONE
    }
}
