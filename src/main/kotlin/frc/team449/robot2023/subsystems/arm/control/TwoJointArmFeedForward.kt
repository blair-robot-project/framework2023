package frc.team449.robot2023.subsystems.arm.control

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Matrix.mat
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.math.numbers.N4
import frc.team449.robot2023.constants.subsystem.ArmConstants
import kotlin.math.cos
import kotlin.math.sign
import kotlin.math.sin

class TwoJointArmFeedForward(
  lengths: Pair<Double, Double>,
  masses: Pair<Double, Double>,
  distanceFromPivot: Pair<Double, Double>,
  private val ks: Pair<Double, Double>,
  private val kv: Pair<Double, Double>,
  ka: Pair<Double, Double>,
  kg: Pair<Double, Double>,
  private val kg1: Pair<Double, Double>,
  private val kg2: Pair<Double, Double>
) {
  private val gravity = 9.8
  private val m1 = masses.first
  private val m2 = masses.second
  private val r1 = distanceFromPivot.first
  private val r2 = distanceFromPivot.second
  private val l1 = lengths.first
  private val h = m2 * l1 * r2
  private val b11 = (m1 * r1 + m2 * (l1 + r2)) * gravity / kg.first
  private val b22 = m2 * r2 * gravity / kg.second
  private val i2 = b22 * ka.second - m2 * r2 * r2
  private val i1 = b11 * ka.first - m1 * r1 * r1 - m2 * (l1 * l1 + r2 * r2) - i2 - 2 * h

  /**
   * Computes the voltage for each joint based on a desired state
   * @param reference the matrix of the desired state matrix in form [theta, beta, theta-dot, beta-dot]
   * @param accelReference desired acceleration for the two joints in form [thetaAccel, betaAccel]
   * @return voltage matrix for the two joint motors in form [joint1Voltage, joint2Voltage] A.K.A. u
   */
  fun calculate(reference: Matrix<N4, N1>, accelReference: Matrix<N2, N1>, singleJointCharacterized: Boolean): Matrix<N2, N1> {
    /** slice the reference state matrix in half */
    // [theta, beta]
    val angles = reference.block<N2, N1>(2, 1, 0, 0)
    // [theta-dot, beta-dot]
    val angularVelocities = reference.block<N2, N1>(2, 1, 2, 0)
    val theta = angles[0, 0]
    val beta = angles[1, 0]
    val thetaDot = angularVelocities[0, 0]
    val betaDot = angularVelocities[1, 0]

    /** pre-computed cosines and sines */
    val c1 = cos(theta)
    val c2 = cos(beta)
    val s2 = sin(beta)
    val c12 = cos(theta + beta)

    /** matrix builders for 2x1 and 2x2 */
    val builder2x1 = mat(N2.instance, N1.instance)
    val builder2x2 = mat(N2.instance, N2.instance)

    /** M matrix in equation: Inertia */
    val M = builder2x2.fill(
      (m1 * r1 * r1 + m2 * (l1 * l1 + r2 * r2) + i1 + i2 + 2 * h * c2),
      (m2 * r2 * r2 + i2 + h * c2),
      (m2 * r2 * r2 + i2 + h * c2),
      (m2 * r2 * r2 + i2)
    )

    /** C matrix in equation: Centrifugal and Coriolis forces */
    val C = builder2x2.fill(
      (-h * s2 * betaDot),
      (-h * s2 * (thetaDot + betaDot)),
      (h * s2 * thetaDot),
      (0.0)
    )

    /** Tau g matrix in equation: Torque on each joint */
    val Tg = builder2x1.fill(
      (gravity * c1) * (m1 * r1 + m2 * l1) + m2 * r2 * gravity * c12,
      m2 * r2 * gravity * c12
    )

    /** B matrix in equation: Motor torque */
    val B = builder2x2.fill(
      b11,
      0.0,
      0.0,
      b22
    )

    /** Kb matrix in equation: Back-emf */
    val Kb = builder2x2.fill(
      b11 * kv.first,
      0.0,
      0.0,
      b22 * kv.second
    )

    /** Ks matrix in equation: Overcome static friction */
    val Ks = builder2x1.fill(
      ks.first * sign(thetaDot),
      ks.second * sign(betaDot)
    )

    /** cos matrix to multiply B^-1(Tg) term */
    val cos = builder2x1.fill(
      c1,
      c12
    )
    val G = builder2x2.fill(
      kg1.first,
      kg1.second,
      kg2.first,
      kg2.second
    )

    val tau = G * cos

    /** Solve equation
     * @see <a href = "https://www.chiefdelphi.com/uploads/short-url/pfucQonJecNeM7gvH57SpOOgPyR.pdf">White paper</a>
     */
    val dTimesAccel = M * accelReference
    val cTimesVel = C * angularVelocities
    val kbTimesVel = Kb * angularVelocities

    /** return u = Km ^ -1 * [D * accel + C * vel + Tg + Kb * vel] */
    return if (singleJointCharacterized) {
      B.solve(dTimesAccel + cTimesVel + Tg + kbTimesVel) + Ks
    }
    /** return u = b^-1(tg) + b^-1(M + C + Kb) + kS */
    else {
      tau + B.solve(dTimesAccel + cTimesVel + kbTimesVel) + Ks
    }
  }

  /**
   * Computes the voltage for each joint based on a desired state with no acceleration
   * @param reference the matrix of the desired state matrix in form [theta, beta, theta-dot, beta-dot]
   * @return voltage matrix for the two joint motors in form [joint1Voltage, joint2Voltage] A.K.A. u
   */
  fun calculate(reference: Matrix<N4, N1>, singleJointCharacterized: Boolean): Matrix<N2, N1> {
    val acceleration = mat(N2.instance, N1.instance).fill(
      0.0,
      0.0
    )
    return calculate(reference, acceleration, singleJointCharacterized)
  }

  companion object {
    /**
     * Uses [ArmConstants] to create an arm feed forward object
     */
    fun createFromConstants(): TwoJointArmFeedForward {
      return TwoJointArmFeedForward(
        ArmConstants.LENGTH_1 to ArmConstants.LENGTH_2,
        ArmConstants.MASS_1 to ArmConstants.MASS_2,
        ArmConstants.R1 to ArmConstants.R2,
        ArmConstants.KS1 to ArmConstants.KS2,
        ArmConstants.KV1 to ArmConstants.KV2,
        ArmConstants.KA1 to ArmConstants.KA2,
        ArmConstants.KG1 to ArmConstants.KG2,
        ArmConstants.KG11 to ArmConstants.KG12,
        ArmConstants.KG21 to ArmConstants.KG22
      )
    }
  }
}
