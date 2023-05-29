package frc.team449.control.holonomic

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team449.control.DriveSubsystem
import frc.team449.control.OI
import frc.team449.robot2023.constants.RobotConstants
import kotlin.math.PI

class TurnCommand(
  private val drive: DriveSubsystem,
  private val oi: OI,
  private val controller: PIDController,
  private val setpoint: Double
) : CommandBase() {
  init {
    addRequirements(drive)
    controller.enableContinuousInput(-PI, PI)
    controller.setSetpoint(setpoint)
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
