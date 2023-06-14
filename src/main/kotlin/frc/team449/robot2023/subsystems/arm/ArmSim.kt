package frc.team449.robot2023.subsystems.arm

import edu.wpi.first.math.geometry.Rotation2d
import frc.team449.robot2023.constants.subsystem.ArmConstants
import frc.team449.robot2023.subsystems.arm.control.ArmEncoder
import frc.team449.robot2023.subsystems.arm.control.ArmPDController
import frc.team449.robot2023.subsystems.arm.control.ArmState
import frc.team449.robot2023.subsystems.arm.control.TwoJointArmFeedForward
import frc.team449.system.encoder.Encoder
import frc.team449.system.encoder.QuadEncoder
import frc.team449.system.motor.WrappedMotor
import frc.team449.system.motor.createSparkMax
import kotlin.math.PI

/**
 * Simulate an arm to check logic of code or visualize trajectory
 */
class ArmSim(
  firstJoint: WrappedMotor,
  secondJoint: WrappedMotor,
  firstJointEncoder: QuadEncoder,
  secondJointEncoder: QuadEncoder,
  feedForward: TwoJointArmFeedForward,
  controller: ArmPDController,
  firstToSecondJoint: Double,
  secondJointToEndEffector: Double
) : Arm(firstJoint, secondJoint, firstJointEncoder, secondJointEncoder, feedForward, controller, firstToSecondJoint, secondJointToEndEffector) {

  private val firstJointEnc = Encoder.SimController(firstJoint.encoder)
  private val secondJointEnc = Encoder.SimController(secondJoint.encoder)
  override var state: ArmState
    get() =
      ArmState(
        Rotation2d(firstJointEnc.position),
        Rotation2d(secondJointEnc.position),
        firstJointEnc.velocity,
        secondJointEnc.velocity
      )
    set(value) {
      desiredState = value
    }

  override fun periodic() {
    super.periodic()
    firstJointEnc.velocity = desiredState.thetaVel
    secondJointEnc.velocity = desiredState.betaVel
    firstJointEnc.position = desiredState.theta.radians
    secondJointEnc.position = desiredState.beta.radians
    firstJointEnc.position = firstJointEnc.position + firstJointEnc.velocity * .02
    secondJointEnc.position = secondJointEnc.position + secondJointEnc.velocity * .02
    visual.setState(state, desiredState)
  }

  companion object {
    fun createArmSim(): ArmSim {
      val firstJointMotor = createSparkMax(
        "First Joint Motor",
        ArmConstants.FIRST_MOTOR_ID1,
        ArmEncoder.creator(
          ArmConstants.FIRST_ENCODER_CHAN,
          ArmConstants.FIRST_ENCODER_OFFSET,
          true
        ),
        slaveSparks = mapOf(
          ArmConstants.FIRST_MOTOR_ID2 to true
        ),
        currentLimit = ArmConstants.FIRST_JOINT_CURR_LIM,
        inverted = true,
        enableBrakeMode = true
      )

      val secondJointMotor = createSparkMax(
        "Second Joint Motor",
        ArmConstants.SECOND_MOTOR_ID,
        ArmEncoder.creator(
          ArmConstants.SECOND_ENCODER_CHAN,
          ArmConstants.SECOND_ENCODER_OFFSET,
          inverted = true
        ),
        currentLimit = ArmConstants.SECOND_JOINT_CURR_LIM,
        enableBrakeMode = true
      )

      val firstJointEncoder = QuadEncoder(
        "First joint quad",
        ArmConstants.FIRST_JOINT_QUAD_ENCODER,
        1024,
        2 * PI,
        1.0
      )

      val secondJointEncoder = QuadEncoder(
        "Second joint quad",
        ArmConstants.SECOND_JOINT_QUAD_ENCODER,
        1024,
        2 * PI,
        1.0
      )

      return ArmSim(
        firstJointMotor,
        secondJointMotor,
        firstJointEncoder,
        secondJointEncoder,
        TwoJointArmFeedForward.createFromConstants(),
        ArmPDController(
          ArmConstants.kP1,
          ArmConstants.kP2,
          ArmConstants.kD1,
          ArmConstants.kD2,
          ArmConstants.kI1,
          ArmConstants.kI2,
          ArmConstants.kErrDeadband
        ),
        ArmConstants.LENGTH_1,
        ArmConstants.LENGTH_2
      )
    }
  }
}
