package frc.team449.control

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.CommandBase

/**
 * Generic driving command that applies the OI output to the Drive
 */
class DriveCommand(
  private val drive: DriveSubsystem,
  private val oi: OI
) : CommandBase() {

  init {
    addRequirements(drive)
  }

  /**
   * Feed [ChassisSpeeds] from the joystick/[OI] to drive[DriveSubsystem]
   */
  override fun execute() {
    drive.set(oi.get())
  }
}
