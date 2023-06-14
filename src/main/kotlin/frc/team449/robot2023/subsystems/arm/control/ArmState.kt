package frc.team449.robot2023.subsystems.arm.control

import edu.wpi.first.math.MatBuilder
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N4

/** Describes the arm with the joint angles */
data class ArmState(
  var theta: Rotation2d = Rotation2d(),
  var beta: Rotation2d = Rotation2d(),
  var thetaVel: Double = 0.0,
  var betaVel: Double = 0.0
) {
  val matrix: Matrix<N4, N1>
    get() {
      val builder = MatBuilder(N4.instance, N1.instance)
      return builder.fill(theta.radians, beta.radians, thetaVel, betaVel)
    }

  companion object

  fun static(): ArmState {
    return ArmState(this.theta, this.beta)
  }

  override fun toString(): String {
    return "(Joint 1: $theta, Joint 2 : $beta, theta speed : $thetaVel, beta speed $betaVel)"
  }
}
