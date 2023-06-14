package frc.team449.robot2023.constants.subsystem

import kotlin.math.PI

object EndEffectorConstants {
  /** Piston and IR constants */
  const val FORWARD_CHANNEL = 5
  const val REVERSE_CHANNEL = 4
  const val SENSOR_CHANNEL = 10

  /** Motor constants */
  const val MOTOR_ID = 25
  const val MOTOR_UPR = 2 * PI
  const val MOTOR_GEARING = 1 / 5.0
  const val MOTOR_CURR_LIM = 20

  /** Voltage constants */
  var INTAKE_VOLTAGE = 12.0
  const val REVERSE_INTAKE_VOLTAGE = -1.75
  const val HOLD_VOLTAGE = 1.95
}
