package frc.team449

import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import frc.team449.control.OI
import frc.team449.control.holonomic.HolonomicDrive

abstract class RobotBase {

  val field = Field2d()

  abstract val powerDistribution: PowerDistribution

  abstract val drive: HolonomicDrive?

  abstract val oi: OI
}
