package frc.team449.robot2023.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team449.robot2023.subsystems.arm.Arm

class ArmCalibration(
  private val arm: Arm,
  private val numSamples: Int = 150
) : CommandBase() {

  init {
    addRequirements(arm)
  }

  private var firstJointSamples = mutableListOf<Double>()
  private var secondJointSamples = mutableListOf<Double>()

  override fun execute() {
    firstJointSamples.add(arm.firstJoint.position)
    secondJointSamples.add(arm.secondJoint.position)
  }

  override fun isFinished(): Boolean {
    return firstJointSamples.size >= numSamples
  }

  override fun end(interrupted: Boolean) {
    firstJointSamples.sort()
    secondJointSamples.sort()
    val firstJointPos = firstJointSamples[(firstJointSamples.size * .9).toInt()]
    val secondJointPos = secondJointSamples[(secondJointSamples.size * .9).toInt()]
    arm.firstJointEncoder.resetPosition(firstJointPos)
    arm.secondJointEncoder.resetPosition(secondJointPos)
    println("***** Finished Calibrating Quadrature reading *****")
  }
}
