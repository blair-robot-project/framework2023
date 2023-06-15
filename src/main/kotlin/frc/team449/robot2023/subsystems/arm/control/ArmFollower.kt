package frc.team449.robot2023.subsystems.arm.control

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team449.robot2023.subsystems.arm.Arm

open class ArmFollower(
  private val arm: Arm,
  private val trajectoryFun: () -> ArmTrajectory?
) : CommandBase() {

  init {
    addRequirements(arm)
  }

  val timer = Timer()
  private var trajectory: ArmTrajectory? = null
  override fun initialize() {
    trajectory = trajectoryFun()
    timer.reset()
    timer.start()
  }

  override fun execute() {
    val currTime = timer.get()
    if (trajectory != null) {
      val reference: ArmState = trajectory!!.sample(currTime)
      arm.moveToState(reference)
    }
  }

  override fun isFinished(): Boolean {
    return trajectory == null || timer.get() > trajectory!!.totalTime
  }

  override fun end(interrupted: Boolean) {
    timer.stop()
    timer.reset()
    arm.setArmDesiredState(arm.getClosestState(arm.desiredState)!!)
    arm.stop()
  }
}
