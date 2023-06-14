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

class EdgeConeCubeStation(
  robot: Robot,
  position: PositionChooser.Positions,
  isRed: Boolean
) : RoutineStructure {

  override val routine =
    HolonomicRoutine(
      drive = robot.drive,
      eventMap = hashMapOf(
        "dropCone" to ArmFollower(robot.arm) { ArmPaths.stowHigh }.andThen(AutoUtil.dropCone(robot)),
        "stowArm" to AutoUtil.deployCube(robot),
        "stopIntake" to AutoUtil.retractGroundIntake(robot),
        "stopIntake2" to AutoUtil.retractGroundIntake(robot),
        "stowArm2" to AutoUtil.deployCube(robot),
        "highArm" to ArmFollower(robot.arm) { ArmPaths.cubeHigh },
        "dropCube" to AutoUtil.dropCube(robot),
        "balanceStation" to AutoBalance.create(robot.drive)
      )
    )

  override val trajectory =
    if (position == PositionChooser.Positions.FARCONE) {
      if (isRed) AutoUtil.transformForAlliance(Paths.FAR.CONECUBESTATION) { true } else Paths.FAR.CONECUBESTATION
    } else {
      if (isRed) AutoUtil.transformForAlliance(Paths.WALL.CONECUBESTATION) { true } else Paths.WALL.CONECUBESTATION
    }
}
