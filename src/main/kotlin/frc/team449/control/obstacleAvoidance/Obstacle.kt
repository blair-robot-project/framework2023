package frc.team449.control.obstacleAvoidance

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import kotlin.math.roundToInt

class Obstacle(xPoints: DoubleArray, yPoints: DoubleArray) {
  var polygon: PolygonDouble

  init {
    polygon = PolygonDouble(xPoints, yPoints)
  }

  fun addNodes(nodes: VisGraph) {
    for (i in 0 until polygon.npoints) {
      nodes.addNode(Node(polygon.xpoints[i], polygon.ypoints[i]))
    }
  }

  /**
   * Creates a polygon that's offset by the distance passed in.
   *
   * Makes a copy of each point of the polygon, offset by a vector
   * at 90deg from the angle of the edges, then connects them together.
   *
   * Has functionality in place to prevent issues with concave shapes
   * having overlapping lines and other weirdness.
   *
   * @param distance Distance to expand the shape outwards by.
   * @return New obstacle, which has the distance passed in added to all sides.
   */
  fun offset(distance: Double): Obstacle {
    // Get a list of all edges with the offsets added onto them
    val offsetEdges: MutableList<ObstacleEdge> = ArrayList()
    for (i in 0 until polygon.npoints) {
      offsetEdges.add(
        ObstacleEdge(
          polygon.xpoints[i],
          polygon.ypoints[i],
          polygon.xpoints[(i + 1) % polygon.npoints],
          polygon.ypoints[(i + 1) % polygon.npoints]
        ).offset(distance)
      )
    }
    val xPoints: MutableList<Double> = ArrayList()
    val yPoints: MutableList<Double> = ArrayList()

    // Loop through all edges, checking if there's an intersection between any of
    // them.
    // If an intersection does occur, cut the point off at that intersection,
    // Creating a new point at the intersection, which should be the correct
    // distance away.
    for (i in offsetEdges.indices) {
      val edge = offsetEdges[i]
      // Check against every other edge, including last -> first
      for (j in i + 1..offsetEdges.size) {
        // Wrap edges back to beginning so the last edge can be checked against the
        // first one
        val otherEdge = offsetEdges[j % offsetEdges.size]
        val intersectionPoint = edge.findIntersectionPoint(otherEdge)
        if (intersectionPoint == null && !edge.hasBeenPlotted()) {
          // If lines don't intersect, and the edge hasn't been plotted out already
          if (!edge.hasBeenCleaned()) {
            // If edge was cleaned from a previous loop, don't add the first point
            xPoints.add(offsetEdges[i].point1.x)
            yPoints.add(offsetEdges[i].point1.y)
          }
          xPoints.add(offsetEdges[i].point2.x)
          yPoints.add(offsetEdges[i].point2.y)
          edge.setHasBeenPlotted(true)
        } else {
          // If lines do intersect
          if (!edge.hasBeenPlotted()) {
            // Don't duplicate points
            if (!edge.hasBeenCleaned()) {
              // If edge was cleaned from a previous loop, don't add the first point
              xPoints.add(offsetEdges[i].point1.x)
              yPoints.add(offsetEdges[i].point1.y)
            }
            xPoints.add(intersectionPoint!!.x)
            yPoints.add(intersectionPoint.y)
            otherEdge.setHasBeenCleaned(true)
            edge.setHasBeenPlotted(true)
          }
        }
      }
    }
    return Obstacle(xPoints.toDoubleArray(), yPoints.toDoubleArray())
  }

  override fun toString(): String {
    var output = "Polygon(\n"
    for (i in 0 until polygon.npoints) {
      output += ((polygon.xpoints[i] * 100).roundToInt() / 100.0).toString() + ", "
      output += ((polygon.ypoints[i] * 100).roundToInt() / 100.0).toString() + "\n"
    }
    return "$output)"
  }

  private inner class ObstacleEdge {
    var point1: Translation2d
    var point2: Translation2d
    private var hasBeenCleaned = false
    private var hasBeenPlotted = false

    constructor(x1: Double, y1: Double, x2: Double, y2: Double) {
      point1 = Translation2d(x1, y1)
      point2 = Translation2d(x2, y2)
    }

    constructor(point1: Translation2d, point2: Translation2d) {
      this.point1 = point1
      this.point2 = point2
    }

    fun offset(distance: Double): ObstacleEdge {
      // Calculate angle of edge
      val angle = point2.minus(point1).angle
      // Calculate perpendicular angle
      val transformAngle = angle.plus(Rotation2d.fromDegrees(90.0))
      // Create offset vector using distance and angles
      val offset = Translation2d(distance, 0.0).rotateBy(transformAngle)
      // Add offset to points
      return ObstacleEdge(
        point1.plus(offset),
        point2.plus(offset)
      )
    }

    /**
     * This is heavily based on an algorithm in the
     * "Tricks of the Windows Game Programming Gurus" book by Andre LeMothe
     *
     * @param other
     * @return
     */
    fun findIntersectionPoint(other: ObstacleEdge): Translation2d? {
      // Calculate x distance of line 1
      val s1_x = point2.x - point1.x
      // Calculate x distance of line 2
      val s2_x = other.point2.x - other.point1.x
      // Calculate y distance of line 1
      val s1_y = point2.y - point1.y
      // Calculate y distance of line 2
      val s2_y = other.point2.y - other.point1.y
      val s: Double
      val t: Double
      // Denominator portion of below equations, split into variable because it's the
      // same between the two
      val d = -s2_x * s1_y + s1_x * s2_y

      // Magical math that I need to look into how it works more
      s = (-s1_y * (point1.x - other.point1.x) + s1_x * (point1.y - other.point1.y)) / d
      t = (s2_x * (point1.y - other.point1.y) - s2_y * (point1.x - other.point1.x)) / d
      val i_x: Double
      val i_y: Double
      if (s in 0.0..1.0 && t in 0.0..1.0) {
        // Intersection found
        i_x = point1.x + t * s1_x
        i_y = point1.y + t * s1_y
        return Translation2d(i_x, i_y)
      }
      return null
    }

    override fun toString(): String {
      return "$point1,$point2"
    }

    fun hasBeenCleaned(): Boolean {
      return hasBeenCleaned
    }

    fun setHasBeenCleaned(hasBeenCleaned: Boolean) {
      this.hasBeenCleaned = hasBeenCleaned
    }

    fun hasBeenPlotted(): Boolean {
      return hasBeenPlotted
    }

    fun setHasBeenPlotted(hasBeenPlotted: Boolean) {
      this.hasBeenPlotted = hasBeenPlotted
    }
  }
}
