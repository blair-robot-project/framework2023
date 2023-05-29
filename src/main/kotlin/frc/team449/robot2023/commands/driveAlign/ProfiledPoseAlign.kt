package frc.team449.robot2023.commands.driveAlign

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team449.control.holonomic.HolonomicDrive
import frc.team449.robot2023.auto.AutoConstants
import frc.team449.robot2023.constants.RobotConstants

/**
 * @param drive The holonomic drive you want to align with
 * @param targetPose The pose you want to drive up to
 * @param xPID The profiled PID controller with constraints you want to use for fixing X error
 * @param yPID The profiled PID controller with constraints you want to use for fixing Y error
 * @param headingPID The non-Profiled PID controller you want to use for fixing rotational error
 * @param tolerance The allowed tolerance from the targetPose
 */
class ProfiledPoseAlign(
  private val drive: HolonomicDrive,
  private val targetPose: Pose2d,
  private val xPID: ProfiledPIDController = ProfiledPIDController(
    AutoConstants.DEFAULT_X_KP,
    0.0,
    0.0,
    TrapezoidProfile.Constraints(4.0, 3.0)
  ),
  private val yPID: ProfiledPIDController = ProfiledPIDController(
    AutoConstants.DEFAULT_Y_KP,
    0.0,
    0.0,
    TrapezoidProfile.Constraints(4.0, 3.0)
  ),
  private val headingPID: ProfiledPIDController = ProfiledPIDController(
    AutoConstants.DEFAULT_ROTATION_KP,
    0.0,
    0.0,
    TrapezoidProfile.Constraints(RobotConstants.MAX_ROT_SPEED, RobotConstants.RATE_LIMIT)
  ),
  private val tolerance: Pose2d = Pose2d(0.05, 0.05, Rotation2d(0.05))
) : CommandBase() {
  init {
    addRequirements(drive)
  }

  override fun initialize() {
    headingPID.enableContinuousInput(-Math.PI, Math.PI)

    // Set tolerances from the given pose tolerance
    xPID.setTolerance(tolerance.x)
    yPID.setTolerance(tolerance.y)
    headingPID.setTolerance(tolerance.rotation.radians)

    // Set the goals for all the PID controller to the target pose
    xPID.setGoal(targetPose.x)
    yPID.setGoal(targetPose.y)
    headingPID.setGoal(targetPose.rotation.radians)
  }

  override fun execute() {
    // Calculate the feedback for X, Y, and theta using their respective controllers
    val xFeedback = xPID.calculate(drive.pose.x)
    val yFeedback = yPID.calculate(drive.pose.y)
    val headingFeedback = headingPID.calculate(drive.heading.radians)

    drive.set(
      ChassisSpeeds.fromFieldRelativeSpeeds(
        xFeedback,
        yFeedback,
        headingFeedback,
        drive.heading
      )
    )
    println(xFeedback)
    println(yFeedback)
    println(headingFeedback)
    println(xPID.goal)
  }

  override fun isFinished(): Boolean {
    return xPID.atGoal() && yPID.atGoal() && headingPID.atGoal()
  }

  override fun end(interrupted: Boolean) {
    drive.stop()
  }
}
