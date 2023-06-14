package frc.team449.robot2023.auto.routines

import com.pathplanner.lib.PathPlannerTrajectory
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.team449.control.auto.HolonomicRoutine
import frc.team449.control.auto.RoutineStructure
import frc.team449.robot2023.Robot
import frc.team449.robot2023.auto.AutoUtil
import frc.team449.robot2023.auto.Paths
import frc.team449.robot2023.auto.PositionChooser
import frc.team449.robot2023.subsystems.arm.ArmPaths
import frc.team449.robot2023.subsystems.arm.control.ArmFollower
import frc.team449.robot2023.subsystems.arm.control.ArmState

class EdgeConeCube(
  robot: Robot,
  position: PositionChooser.Positions,
  isRed: Boolean
) : RoutineStructure {

  override val routine =
    HolonomicRoutine(
      drive = robot.drive,
      eventMap = hashMapOf(
        "dropCone" to AutoUtil.stowDropCone(robot),
        "stowArm" to AutoUtil.deployCube(robot),
        "stopIntake" to AutoUtil.retractGroundIntake(robot),
        "dropCube" to ArmFollower(robot.arm) { ArmPaths.cubeHigh }.andThen(AutoUtil.dropCube(robot)).andThen(
          WaitCommand(0.5)
        ).andThen(
          ArmFollower(robot.arm) { ArmPaths.highStow }
        ),
        "stopIntake2" to AutoUtil.retractAndStow(robot),
        "stowCube" to AutoUtil.deployCube(robot).andThen(
          InstantCommand({
            robot.arm.moveToState(
              ArmState(
                Rotation2d.fromDegrees(74.15),
                Rotation2d.fromDegrees(-132.17)
              )
            )
          })
        ),
        "retractCube" to AutoUtil.retractAndStow(robot),
        "highCube" to ArmFollower(robot.arm) { ArmPaths.stowHigh }
          .andThen(WaitCommand(0.1))
          .andThen(AutoUtil.dropCube(robot))
      )
    )

  override val trajectory: MutableList<PathPlannerTrajectory> =
    if (position == PositionChooser.Positions.FARCONE) {
      if (isRed) AutoUtil.transformForAlliance(Paths.FAR.CONECUBE) { true } else Paths.FAR.CONECUBE
    } else {
      if (isRed) AutoUtil.transformForAlliance(Paths.WALL.CONECUBE) { true } else Paths.WALL.CONECUBE
    }
}
