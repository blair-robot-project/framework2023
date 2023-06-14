package frc.team449.robot2023.subsystems.arm.control

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Matrix.mat
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N2
import org.ejml.data.SingularMatrixException
import kotlin.math.*

/**
 * Forward and inverse kinematics solver for two-jointed arms
 * @param pivotToJointLengthMeters length from first base pivot to joint in meters
 * @param jointToEndEffectorLengthMeters length from joint to end effector in meters
 */
class ArmKinematics(
  private val pivotToJointLengthMeters: Double,
  private val jointToEndEffectorLengthMeters: Double
) {

  /**
   *  <theta, beta, theta-dot, beta-dot> -> <x, z, x-dot, z-dot>
   *  @param armState the state of the joints to be converted from
   */
  fun toCartesian(
    armState: ArmState
  ): CartesianArmState {
    val l1 = pivotToJointLengthMeters
    val l2 = jointToEndEffectorLengthMeters
    val theta = armState.theta
    val beta = armState.beta
    val c1 = theta.cos
    val s1 = theta.sin
    val c12 = (theta + beta).cos
    val s12 = (theta + beta).sin

    /** compute cartesian positions */
    val x = l1 * c1 + l2 * c12
    val z = l1 * s1 + l2 * s12

    /** compute cartesian velocities */
    val angular = mat(N2.instance, N1.instance)
      .fill(armState.thetaVel, armState.betaVel)

    /** The jacobian matrix in x-dot = J * q-dot */
    val jacobian = jacobian(l1, l2, c1, s1, c12, s12)

    /** linear velocities in form [x-dot, z-dot] */
    val linear = jacobian * angular
    val xSpeed = linear[0, 0]
    val zSpeed = linear[1, 0]

    return CartesianArmState(x, z, xSpeed, zSpeed)
  }

  /**
   * <x, z, x-dot, z-dot> -> <theta, beta, theta-dot, beta-dot>
   * @param cartesianState the coordinate state to convert from
   * @param currentState the arm's current state of the joints used to return the most optimal joints state
   * @return angles for the joints given the cartesian coordinate
   */
  fun toAngularState(
    cartesianState: CartesianArmState,
    currentState: ArmState
  ): ArmState? {
    val l1 = pivotToJointLengthMeters
    val l2 = jointToEndEffectorLengthMeters
    val x = cartesianState.x
    val z = cartesianState.z
    val xSpeed = cartesianState.xSpeed
    val zSpeed = cartesianState.zSpeed
    val armTheta = currentState.theta.radians
    val armBeta = currentState.beta.radians

    /** Step 1: find both possible solutions for joint angles */

    // first solution when arm is open in CCW direction ⏋
    val beta1 = acos((x * x + z * z - l1 * l1 - l2 * l2) / (2 * l1 * l2))
    val theta1 = atan2(z, x) - atan2(l2 * sin(beta1), l1 + l2 * cos(beta1))
    // second solution when arm is open in CW direction ⎾
    val beta2 = -beta1
    val theta2 = atan2(z, x) + atan2(l2 * sin(beta1), l1 + l2 * cos(beta1))

    /** Return null when the arm cannot reach this coordinate */
    if (beta1.isNaN() || beta2.isNaN() || theta1.isNaN() || theta2.isNaN()) return null

    /** Step 2: decide which solution requires less movement
     * Costs are calculated based on how much "movement" the joints have to make.
     * Prioritizes not rotating the base pivot
     * */
    val cost1 = (l1 + l2) * abs(theta1 - armTheta) + (l2) * abs(beta1 - armBeta)
    val cost2 = (l1 + l2) * abs(theta2 - armTheta) + (l2) * abs(beta2 - armBeta)

    // minimum cost solution is chosen
    val theta: Rotation2d
    val beta: Rotation2d
    if (cost1 < cost2) {
      // pick solution 1
      theta = Rotation2d(theta1)
      beta = Rotation2d(beta1)
    } else {
      // pick solution 2
      theta = Rotation2d(theta2)
      beta = Rotation2d(beta2)
    }

    /** Step 3: Find the velocities for the joints */
    /** Using the equation x = J * q
     * we solve for q, the matrix of joint velocities */
    // pre-compute cosines and sines
    val c1 = theta.cos
    val s1 = theta.sin
    val c12 = (theta + beta).cos
    val s12 = (theta + beta).sin

    val jacobian = jacobian(l1, l2, c1, s1, c12, s12)

    val linear = mat(N2.instance, N1.instance).fill(
      xSpeed,
      zSpeed
    )

    val angular: Matrix<N2, N1>

    /** [q = pseudoInverse(J) * x], catch singularities */
    try {
      angular = jacobian.solve(linear)
    } catch (e: SingularMatrixException) {
      return null
    }

    val thetaVel = angular[0, 0]
    val betaVel = angular[1, 0]

    return ArmState(theta, beta, thetaVel, betaVel)
  }

  companion object {
    /**
     * Constructs the jacobian matrix from some parameters
     * @see <a href="https://robotacademy.net.au/lesson/velocity-of-2-joint-planar-robot-arm/">this video</a>
     * @param l1 length from pivot to joint in meters
     * @param l2 length from joint to end effector in meters
     * @param c1 cosine of pivot angle
     * @param s1 sine of pivot angle
     * @param c12 cosine of the sum of the pivot angle and joint angle
     * @param s12 sine of the sum of the pivot angle and joint angle
     * @return Jacobian Matrix
     */
    fun jacobian(
      l1: Double,
      l2: Double,
      c1: Double,
      s1: Double,
      c12: Double,
      s12: Double
    ): Matrix<N2, N2> {
      return mat(N2.instance, N2.instance).fill(
        -(l1 * s1 + l2 * s12),
        -l2 * s12,
        l1 * c1 + l2 * c12,
        l2 * c12
      )
    }
  }
}
