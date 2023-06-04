package frc.team449.control.holonomic

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team449.control.DriveSubsystem
import frc.team449.control.OI
import frc.team449.robot2023.constants.RobotConstants
import kotlin.math.PI

/**
 * Make the robot face a specified angle.
 * @param drive The robot's drivetrain.
 * @param oi The OI to control the drivetrain.
 * @param controller The PID controller to adjust the robot's heading.
 * @param angleSetpoint The desired heading of the robot in radians.
 */
class TurnCommand(
  private val drive: DriveSubsystem,
  private val oi: OI,
  private val controller: PIDController,
  private val angleSetpoint: Double
) : CommandBase() {
  init {
    addRequirements(drive)
    controller.enableContinuousInput(-PI, PI)
    controller.setSetpoint(angleSetpoint)
    controller.setTolerance(0.1)
  }

  override fun execute() {
    val inputs = ChassisSpeeds(
      oi.get().vxMetersPerSecond,
      oi.get().vyMetersPerSecond,
      controller.calculate(drive.heading.radians) * RobotConstants.MAX_ROT_SPEED
    )
    drive.set(inputs)
  }

  override fun isFinished(): Boolean {
    return (controller.atSetpoint())
  }

  // TODO: fix the problem with the robot moving with the rotation
}
