package frc.team449.robot2023.commands.arm

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team449.robot2023.subsystems.arm.Arm
import frc.team449.robot2023.subsystems.arm.control.ArmState
import kotlin.math.abs

class ArmSweep(
  private val arm: Arm,
  private val input: () -> Double,
  private var sweepBeta: Rotation2d
) : CommandBase() {

  private lateinit var startState: ArmState

  init {
    addRequirements(arm)
  }

  override fun initialize() {
    startState = arm.desiredState.copy()
    sweepBeta = if (startState.beta.degrees < 0.0) {
      Rotation2d.fromDegrees(-abs(sweepBeta.degrees))
    } else {
      Rotation2d.fromDegrees(abs(sweepBeta.degrees))
    }
  }

  override fun execute() {
    arm.moveToState(
      ArmState(
        startState.theta,
        startState.beta + sweepBeta * input(),
        startState.thetaVel,
        startState.betaVel
      )
    )
  }

  override fun isFinished(): Boolean {
    return false
  }
}
