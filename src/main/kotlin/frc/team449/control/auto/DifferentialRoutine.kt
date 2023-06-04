package frc.team449.control.auto

import com.pathplanner.lib.PathPlannerTrajectory
import com.pathplanner.lib.auto.BaseAutoBuilder
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team449.control.differential.DifferentialDrive

/**
 * @param drive The differential drive subsystem to be used when following a trajectory.
 * @param eventMap A hash map of event marker names paired with the command you want to run that cannot require drive
 * @param translationTol Allowed error in meters when following a trajectory
 * @param thetaTol Allowed error in radians when following a trajectory
 * @param resetPosition Whether to reset your position to the initial pose in the first trajectory
 * @param timeout Maximum time in seconds for the path follower to correct itself after EACH trajectory is done
 */
class DifferentialRoutine(
  private val drive: DifferentialDrive,
  eventMap: HashMap<String, Command>,
  private val translationTol: Double = 0.05,
  private val thetaTol: Double = 0.05,
  private val resetPosition: Boolean = false,
  private val timeout: Double = 1.0
) : BaseAutoBuilder(drive::pose, eventMap, DrivetrainType.STANDARD) {

  /** What command you want to use to follow a given trajectory */
  override fun followPath(trajectory: PathPlannerTrajectory): CommandBase {
    return DifferentialFollower(
      drive,
      trajectory,
      false,
      translationTol,
      thetaTol,
      timeout
    )
  }
}
