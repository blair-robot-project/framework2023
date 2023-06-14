package frc.team449.robot2023.constants.subsystem

import kotlin.math.PI

object GroundIntakeConstants {

  const val INTAKE_VOLTAGE = 7.0

  /** Ground Intake Motor Constants */
  const val INTAKE_TOP = 31
  const val INTAKE_BOTTOM = 30
  const val UPR = 2 * PI
  const val GEARING = 1.0 / 3.0
  const val TOP_INVERTED = false
  const val BOTTOM_INVERTED = true
  const val CURRENT_LIM = 20

  const val PISTON_FWD = 7
  const val PISTON_REV = 6
}
