package frc.team449.robot2023.auto.routines

import com.pathplanner.lib.PathPlannerTrajectory
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.team449.control.auto.HolonomicRoutine
import frc.team449.control.auto.RoutineStructure
import frc.team449.robot2023.Robot
import frc.team449.robot2023.auto.AutoUtil
import frc.team449.robot2023.auto.Paths
import frc.team449.robot2023.auto.PositionChooser
import frc.team449.robot2023.subsystems.arm.ArmPaths
import frc.team449.robot2023.subsystems.arm.control.ArmFollower
import frc.team449.robot2023.subsystems.arm.control.ArmState

class EdgeConeCubeCube(
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
        "midCube" to ArmFollower(robot.arm) { ArmPaths.cubeMid }.andThen(
          InstantCommand({
            robot.arm.moveToState(
              ArmState(
                Rotation2d.fromDegrees(85.36),
                Rotation2d.fromDegrees(105.0)
              )
            )
          })
        ),
        "stopIntake" to AutoUtil.retractGroundIntake(robot),
        "dropCone" to AutoUtil.stowDropCone(robot),
        "stopCubeIntake" to AutoUtil.retractGroundIntake(robot),
        "dropCube2" to AutoUtil.dropCube(robot)
      ),
      timeout = 0.25
    )

  override val trajectory: MutableList<PathPlannerTrajectory> =
    if (position == PositionChooser.Positions.FARCONE) {
      if (isRed) AutoUtil.transformForAlliance(Paths.FAR.CONECUBECUBE) { true } else Paths.FAR.CONECUBECUBE
    } else {
      if (isRed) AutoUtil.transformForAlliance(Paths.WALL.CONECUBECUBE) { true } else Paths.WALL.CONECUBECUBE
    }
}
