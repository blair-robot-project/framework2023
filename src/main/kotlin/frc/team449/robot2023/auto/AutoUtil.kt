package frc.team449.robot2023.auto

import com.pathplanner.lib.PathPlannerTrajectory
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.*
import frc.team449.robot2023.Robot
import frc.team449.robot2023.constants.auto.AutoConstants
import frc.team449.robot2023.constants.field.FieldConstants
import frc.team449.robot2023.constants.subsystem.ArmConstants
import frc.team449.robot2023.subsystems.arm.ArmPaths
import frc.team449.robot2023.subsystems.arm.control.ArmFollower
import frc.team449.robot2023.subsystems.arm.control.ArmState
import java.util.Collections
import java.util.function.BooleanSupplier
import kotlin.math.PI

object AutoUtil {
  fun transformForFarSide(trajList: MutableList<PathPlannerTrajectory>): MutableList<PathPlannerTrajectory> {
    val correctedTrajList: MutableList<PathPlannerTrajectory> = MutableList(
      trajList.size
    ) { PathPlannerTrajectory() }

    Collections.copy(correctedTrajList, trajList)

    for ((index, _) in correctedTrajList.withIndex()) {
      for (s in correctedTrajList[index].states) {
        s as PathPlannerTrajectory.PathPlannerState
        s.poseMeters = Pose2d(s.poseMeters.x, AutoConstants.Y_DISTANCE_BARRIER - s.poseMeters.y, -s.poseMeters.rotation)
        s.holonomicAngularVelocityRadPerSec *= -1.0
        s.holonomicRotation *= -1.0
      }
    }

    return correctedTrajList
  }

  fun transformForAlliance(pathGroup: MutableList<PathPlannerTrajectory>, isRed: BooleanSupplier): MutableList<PathPlannerTrajectory> {
    val correctedPathGroup: MutableList<PathPlannerTrajectory> = MutableList(
      pathGroup.size
    ) { PathPlannerTrajectory() }

    Collections.copy(correctedPathGroup, pathGroup)

    if (isRed.asBoolean) {
      for ((index, _) in correctedPathGroup.withIndex()) {
        correctedPathGroup[index] = PathPlannerTrajectory.transformTrajectoryForAlliance(
          correctedPathGroup[index],
          DriverStation.getAlliance()
        )

        for (s in correctedPathGroup[index].states) {
          s as PathPlannerTrajectory.PathPlannerState
          s.poseMeters = Pose2d(
            FieldConstants.fieldLength - s.poseMeters.x,
            FieldConstants.fieldWidth - s.poseMeters.y,
            s.poseMeters.rotation.plus(Rotation2d(PI))
          )
          s.holonomicRotation = s.holonomicRotation.plus(Rotation2d(PI))
        }
      }
    }

    return correctedPathGroup
  }

  fun dropCone(robot: Robot): Command {
    return SequentialCommandGroup(
      RepeatCommand(
        InstantCommand(
          {
            val currState = robot.arm.desiredState.copy()
            robot.arm.moveToState(
              ArmState(
                currState.theta,
                currState.beta + Rotation2d.fromDegrees(AutoConstants.CONE_DROP_SWEEP_SPEED),
                currState.thetaVel,
                currState.betaVel
              )
            )
          }
        )
      ).withTimeout(AutoConstants.CONE_DROP_SWEEP_TIME),
      InstantCommand(robot.endEffector::pistonRev)
    )
  }

  fun dropCube(robot: Robot): Command {
    return SequentialCommandGroup(
      WaitCommand(AutoConstants.CUBE_DROP_WAIT_BEFORE),
      InstantCommand(robot.endEffector::autoReverse),
      WaitCommand(AutoConstants.CUBE_DROP_WAIT_AFTER)
    )
  }

  fun stowDropCube(robot: Robot): Command {
    return InstantCommand(robot.endEffector::holdIntake).andThen(
      ArmFollower(robot.arm) { ArmPaths.stowHigh }.andThen(dropCube(robot))
    )
  }

  fun stowDropCone(robot: Robot): Command {
    return InstantCommand(robot.endEffector::holdIntake).andThen(
      ArmFollower(robot.arm) { ArmPaths.stowHigh }.andThen(dropCone(robot))
    )
  }

  fun deployCube(robot: Robot): Command {
    return SequentialCommandGroup(
      robot.groundIntake.deploy(),
      robot.groundIntake.intakeCube(),
      InstantCommand(robot.endEffector::intake),
      InstantCommand(robot.endEffector::pistonRev),
      ArmFollower(robot.arm) { ArmPaths.highCube }
    )
  }

  fun deployCone(robot: Robot): Command {
    return SequentialCommandGroup(
      InstantCommand(robot.endEffector::intake),
      InstantCommand(robot.endEffector::pistonOn),
      ArmFollower(robot.arm) { ArmPaths.highCone }
    )
  }

  fun stowArm(robot: Robot): Command {
    return ArmFollower(robot.arm) { ArmPaths.highStow }
  }

  fun retractAndStow(robot: Robot): Command {
    return SequentialCommandGroup(
      retractGroundIntake(robot),
      ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.STOW) }
    )
  }

  fun retractGroundIntake(robot: Robot): Command {
    return SequentialCommandGroup(
      InstantCommand(robot.endEffector::strongHoldIntake),
      robot.groundIntake.retract(),
      robot.groundIntake.runOnce(robot.groundIntake::stop)
    )
  }
}
