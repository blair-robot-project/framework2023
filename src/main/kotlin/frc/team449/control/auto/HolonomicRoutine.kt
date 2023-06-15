package frc.team449.control.auto

import com.pathplanner.lib.PathPlannerTrajectory
import com.pathplanner.lib.auto.BaseAutoBuilder
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.*
import frc.team449.control.holonomic.HolonomicDrive
import frc.team449.robot2023.constants.auto.AutoConstants
import io.github.oblarg.oblog.annotations.Config
import kotlin.math.abs

/**
 * @param xController The PID controller used to correct X translation error when following a trajectory
 * @param yController The PID controller used to correct Y translation error when following a trajectory
 * @param thetaController The PID controller used to correct rotational  error when following a trajectory
 * @param drive The holonomic drive subsystem to be used when following a trajectory
 * @param eventMap A hash map of event marker names paired with the command you want to run that cannot require drive
 * @param translationTol Allowed error in meters when following a trajectory
 * @param thetaTol Allowed error in radians when following a trajectory
 * @param resetPosition Whether to reset your position to the initial pose for all parts of the trajectory
 * @param timeout Maximum time in seconds for the path follower to correct itself after EACH trajectory is done
 */
class HolonomicRoutine(
  @field:Config.PIDController(name = "X PID") var xController: PIDController = PIDController(AutoConstants.DEFAULT_X_KP, 0.0, 0.0),
  @field:Config.PIDController(name = "Y PID") var yController: PIDController = PIDController(AutoConstants.DEFAULT_Y_KP, 0.0, 0.0),
  @field:Config.PIDController(name = "Rotation PID") var thetaController: PIDController = PIDController(AutoConstants.DEFAULT_ROTATION_KP, 0.0, 0.0),
  private val drive: HolonomicDrive,
  private val eventMap: HashMap<String, Command>,
  private val translationTol: Double = 0.05,
  private val thetaTol: Double = 0.05,
  private val resetPosition: Boolean = false,
  private val resetPositionTolerance: Pose2d = Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)),
  private val timeout: Double = 0.65
) : BaseAutoBuilder(drive::pose, eventMap, DrivetrainType.HOLONOMIC) {

  /** What command you want to use to follow a given trajectory */
  override fun followPath(traj: PathPlannerTrajectory): CommandBase {
    return HolonomicFollower(
      drive,
      traj,
      xController,
      yController,
      thetaController,
      Pose2d(
        translationTol,
        translationTol,
        Rotation2d(thetaTol)
      ),
      timeout,
      resetPose = resetPosition
    )
  }

  override fun resetPose(trajectory: PathPlannerTrajectory): CommandBase {
    val poseError = drive.pose.relativeTo(trajectory.initialHolonomicPose)
    if (abs(poseError.x) < resetPositionTolerance.x &&
      abs(poseError.y) < resetPositionTolerance.y &&
      abs(poseError.rotation.radians) < resetPositionTolerance.rotation.radians
    ) {
      return InstantCommand()
    }
    return InstantCommand({ drive.pose = trajectory.initialHolonomicPose })
  }

  private fun followTrajEvents(traj: PathPlannerTrajectory): Command {
    val followingEventCommands = SequentialCommandGroup()
    val prevTime = 0.0

    for (marker in traj.markers) {
      followingEventCommands.addCommands(WaitCommand(marker.timeSeconds - prevTime))
      for (name in marker.names) {
        if (eventMap.containsKey(name)) {
          followingEventCommands.addCommands(eventMap[name])
        }
      }
    }

    return followingEventCommands
  }

  fun createRoutine(pathGroup: MutableList<PathPlannerTrajectory>): Command {
    val commands = SequentialCommandGroup()

    commands.addCommands(resetPose(pathGroup[0]))

    for (traj in pathGroup) {
      commands.addCommands(
        SequentialCommandGroup(
          stopEventGroup(traj.startStopEvent),
          ParallelCommandGroup(
            followTrajEvents(traj),
            followPath(traj)
          )
        )
      )
    }

    commands.addCommands(stopEventGroup(pathGroup[pathGroup.size - 1].endStopEvent))

    return commands
  }
}
