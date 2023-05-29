package frc.team449.control.obstacleAvoidance

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d

class Node {
  var x: Double
  var y: Double
  var holRot: Rotation2d
  var neighbors: MutableList<Node>

  constructor(x: Double, y: Double) {
    this.x = x
    this.y = y
    holRot = Rotation2d.fromDegrees(0.0)
    neighbors = ArrayList()
  }

  constructor(pose: Pose2d) {
    x = pose.x
    y = pose.y
    holRot = pose.rotation
    neighbors = ArrayList()
  }

  fun addNeighbor(neighbor: Node) {
    neighbors.add(neighbor)
  }

  fun setHolRot(degree: Double) {
    holRot = Rotation2d.fromDegrees(degree)
  }

  override fun toString(): String {
    return "X Position: $x\tY Position: $y"
  }
}
