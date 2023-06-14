package frc.team449.robot2023.auto.routines

import frc.team449.control.auto.HolonomicRoutine
import frc.team449.control.auto.RoutineStructure
import frc.team449.robot2023.Robot
import frc.team449.robot2023.auto.AutoUtil
import frc.team449.robot2023.auto.Paths
import frc.team449.robot2023.auto.PositionChooser
import frc.team449.robot2023.commands.autoBalance.AutoBalance
import frc.team449.robot2023.subsystems.arm.ArmPaths
import frc.team449.robot2023.subsystems.arm.control.ArmFollower

class EdgeCubeConeStation(
  robot: Robot,
  position: PositionChooser.Positions,
  isRed: Boolean
) : RoutineStructure {

  override val routine =
    HolonomicRoutine(
      drive = robot.drive,
      eventMap = hashMapOf(
        "dropCone" to AutoUtil.dropCone(robot),
        "stowArm" to AutoUtil.deployCone(robot),
        "stopIntake" to AutoUtil.retractGroundIntake(robot),
        "stopIntake2" to AutoUtil.retractGroundIntake(robot),
        "dropCube" to AutoUtil.stowDropCube(robot),
        "highArm" to ArmFollower(robot.arm) { ArmPaths.coneHigh },
        "balanceStation" to AutoBalance.create(robot.drive),
        "endArm" to AutoUtil.stowArm(robot),
        // TODO: Cone or cube?
        "deployArm" to AutoUtil.deployCone(robot)
      )
    )

  override val trajectory =
    if (position == PositionChooser.Positions.FARCUBE) {
      if (isRed) AutoUtil.transformForAlliance(Paths.FAR.CUBECONESTATION) { true } else Paths.FAR.CUBECONESTATION
    } else {
      if (isRed) AutoUtil.transformForAlliance(Paths.WALL.CUBECONESTATION) { true } else Paths.WALL.CUBECONESTATION
    }
}
