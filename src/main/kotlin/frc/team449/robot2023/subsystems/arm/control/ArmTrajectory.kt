package frc.team449.robot2023.subsystems.arm.control

import edu.wpi.first.math.InterpolatingMatrixTreeMap
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N4
import edu.wpi.first.wpilibj.Filesystem
import org.json.simple.JSONArray
import org.json.simple.JSONObject
import org.json.simple.parser.JSONParser
import java.io.File
import java.io.FileReader

class ArmTrajectory(
  filename: String
) {
  private val path = Filesystem.getDeployDirectory().absolutePath.plus("/trajectories/$filename")
  private var jaMorant = JSONParser().parse(FileReader(File(path).absolutePath)) as JSONObject
  private var stateMap: InterpolatingMatrixTreeMap<Double, N4, N1> = InterpolatingMatrixTreeMap()
  var totalTime: Double = 0.0

  fun parse() {
    var total = 0.0
    val states = jaMorant["states"] as JSONArray
    states.forEach { state ->
      state as JSONObject
      val currTime = state["t"].toString().toDouble()
      total = currTime

      val currArmState = ArmState(
        Rotation2d(state["q1"].toString().toDouble()),
        Rotation2d(state["q2"].toString().toDouble()),
        state["q1d"].toString().toDouble(),
        state["q2d"].toString().toDouble()
      )
      val currArmStateMatrix = currArmState.matrix
      stateMap.put(currTime, currArmStateMatrix)
    }
    totalTime = total
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

  fun sample(timeSeconds: Double): ArmState {
    val t = MathUtil.clamp(timeSeconds, 0.0, totalTime)
    return stateMap.get(t).state()
  }
}
