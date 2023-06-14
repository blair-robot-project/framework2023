package frc.team449.robot2023.subsystems.arm.control

import edu.wpi.first.math.MatBuilder
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N4

/** Describes where the end effector of the arm is in space (x,z) */
data class CartesianArmState(
  val x: Double,
  val z: Double,
  val xSpeed: Double,
  val zSpeed: Double
) {
  val matrix: Matrix<N4, N1>
    get() {
      val builder = MatBuilder(N4.instance, N1.instance)
      return builder.fill(x, z, xSpeed, zSpeed)
    }

  override fun toString(): String {
    return "(x: $x, z : $z, x speed : $xSpeed, z speed $zSpeed)"
  }
}
