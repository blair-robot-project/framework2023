package frc.team449.robot2023.commands

import edu.wpi.first.math.InterpolatingMatrixTreeMap
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N4
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team449.robot2023.subsystems.arm.Arm
import frc.team449.robot2023.subsystems.arm.control.ArmState
import org.json.simple.JSONArray
import org.json.simple.JSONObject
import java.io.FileWriter

class ArmCharacterizer(
  private val arm: Arm,
  dt: Double,
  private val holdTime: Double,
  periods: Int
) : CommandBase() {

  private var holdTimer = Timer()
  private val timer = Timer()
  private var stateMap: InterpolatingMatrixTreeMap<Double, N4, N1> = InterpolatingMatrixTreeMap()
  private var t = 0.0
  private var q1 = 45.0
  private var q2 = 90.0
  private var log = JSONArray()

  init {
    addRequirements(arm)

    while (q1 < 135.0) {
      stateMap.put(
        t,
        ArmState(
          Rotation2d.fromDegrees(q1),
          Rotation2d.fromDegrees(q2)
        ).matrix
      )
      q1 += (90.0) / (periods.toDouble())
      q2 = -q2
      t += dt
    }
  }

  override fun initialize() {
    holdTimer.reset()
    holdTimer.start()
    timer.reset()
    timer.start()
  }

  override fun execute() {
    if (holdTimer.hasElapsed(holdTime)) {
      val currData = JSONObject()

      currData["t"] = timer.get()
      currData["q1"] = arm.state.theta.radians
      currData["q2"] = arm.state.beta.radians
      currData["u1"] = arm.firstJoint.lastVoltage
      currData["u2"] = arm.secondJoint.lastVoltage
      currData["q1-setpoint"] = arm.desiredState.theta.radians
      currData["q2-setpoint"] = arm.desiredState.beta.radians

      log.add(currData)
      arm.moveToState(stateMap.get(timer.get()).state())
      holdTimer.reset()
      holdTimer.start()
    }
    arm.holdArm()
  }

  override fun isFinished(): Boolean {
    return timer.hasElapsed(t)
  }

  override fun end(interrupted: Boolean) {
    println("Starting to Save Characterization File :)")

    val writer = FileWriter("/home/lvuser/characterization_data.json")
    writer.write(log.toJSONString())
    writer.flush()
    writer.close()
    holdTimer.stop()
    holdTimer.reset()

    println("Saved Characterization File :)")
    arm.stop()
  }

  fun Matrix<N4, N1>.state(): ArmState {
    val theta = this[0, 0]
    val beta = this[1, 0]
    val thetaVel = this[2, 0]
    val betaVel = this[3, 0]

    return ArmState(
      Rotation2d(theta),
      Rotation2d(beta),
      thetaVel,
      betaVel
    )
  }
}
