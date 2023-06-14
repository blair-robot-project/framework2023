package frc.team449.robot2023.subsystems.arm.control

import edu.wpi.first.math.MathUtil.applyDeadband
import edu.wpi.first.math.MathUtil.clamp
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Matrix.mat
import edu.wpi.first.math.Nat
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.math.numbers.N4
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import kotlin.math.PI

class ArmPDController(
  private val kP1: Double,
  private val kP2: Double,
  private val kD1: Double,
  private val kD2: Double,
  private val kI1: Double,
  private val kI2: Double,
  private val errDeadband: Double
) : Sendable {
  private var setpoint: Matrix<N4, N1>? = null

  private var errorSum = mat(N4.instance, N1.instance).fill(0.0, 0.0, 0.0, 0.0)

  /**
   * @param state the current state observed of the system from sensors
   * @param reference the desired state where the system should be
   * @return voltage for joint1 and joint2 to correct for error
   */
  fun calculate(state: Matrix<N4, N1>, reference: Matrix<N4, N1>): Matrix<N2, N1> {
    setpoint = reference
    val err = reference - state
    val wrappedErr = mat(N4.instance, N1.instance).fill(
      applyDeadband(err[0, 0], errDeadband * PI / 180.0),
      applyDeadband(err[1, 0], errDeadband * PI / 180.0),
      err[2, 0],
      err[3, 0]
    )

    errorSum = mat(N4.instance, N1.instance).fill(
      errorSum[0, 0] + wrappedErr[0, 0],
      errorSum[1, 0] + wrappedErr[1, 0],
      errorSum[2, 0] + wrappedErr[2, 0],
      errorSum[3, 0] + wrappedErr[3, 0]
    )

    val I = mat(N2.instance, N4.instance).fill(
      kI1,
      0.0,
      0.0,
      0.0,
      0.0,
      kI2,
      0.0,
      0.0
    )

    val K = mat(N2.instance, N4.instance).fill(
      kP1,
      0.0,
      kD1,
      0.0,
      0.0,
      kP2,
      0.0,
      kD2
    )
    val output = K * wrappedErr + I * errorSum

    output[0, 0] = clamp(output[0, 0], -11.5, 11.5) // first joint PID is bounded 11V min -11V
    output[1, 0] = clamp(output[1, 0], -11.5, 11.5) // second joint PID is bounded 11V min -11V

    return output
  }

  /**
   * calculate using saved setpoint from last calculation
   */
  fun calculate(state: Matrix<N4, N1>): Matrix<N2, N1> {
    // !! to assert that set point shouldn't be null
    return calculate(state, setpoint!!)
  }

  fun reset() {
    errorSum = mat(Nat.N4(), Nat.N1()).fill(0.0, 0.0, 0.0, 0.0)
  }

  override fun initSendable(builder: SendableBuilder?) {
    builder!!.addDoubleProperty("err q1", { errorSum[0, 0] }, null)
    builder.addDoubleProperty("err q2", { errorSum[1, 0] }, null)
    builder.addDoubleProperty("err q1d", { errorSum[2, 0] }, null)
    builder.addDoubleProperty("err q2d", { errorSum[3, 0] }, null)
  }
}
