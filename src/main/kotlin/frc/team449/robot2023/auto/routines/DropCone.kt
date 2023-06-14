package frc.team449.robot2023.auto.routines

import com.pathplanner.lib.PathPlannerTrajectory
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.control.auto.HolonomicRoutine
import frc.team449.control.auto.RoutineStructure
import frc.team449.robot2023.Robot
import frc.team449.robot2023.auto.AutoUtil
import frc.team449.robot2023.constants.subsystem.ArmConstants
import frc.team449.robot2023.subsystems.arm.control.ArmFollower

class DropCone(
  private val robot: Robot
) : RoutineStructure {

  override val routine = HolonomicRoutine(
    drive = robot.drive,
    eventMap = hashMapOf()
  )

  override val trajectory = mutableListOf(PathPlannerTrajectory())

  override fun createCommand(robot: Robot): Command {
    return AutoUtil.stowDropCone(robot).andThen(
      ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.STOW) }
    )
  }
}
