package frc.team449.control.obstacleAvoidance

import java.awt.geom.Line2D
import kotlin.math.hypot

class VisGraph {
  // A class representing the navigation mesh
  private val nodes: MutableList<Node>
  private val edges: MutableList<Edge>

  init {
    nodes = ArrayList()
    edges = ArrayList()
  }

  // Add a node to the navigation mesh
  fun addNode(node: Node) {
    if (!containsNode(node)) {
      nodes.add(node)
    }
  }

  val nodeSize: Int
    get() = nodes.size

  fun getNode(index: Int): Node {
    return nodes[index]
  }

  fun containsNode(node: Node): Boolean {
    return nodes.contains(node)
  }

  // Add an edge to the navigation mesh
  fun addEdge(edge: Edge, obstacles: List<Obstacle>): Boolean {
    if (!containsEdge(edge)) {
      // Why not use the Line2D class' static method of .linesIntersect() ? I am just
      // hold on
      for (obstacle in obstacles) {
        val polygon = obstacle.polygon
        for (i in 0 until polygon.npoints) {
          val j = (i + 1) % polygon.npoints
          // Couldn't the mod be eliminated by starting the index i at 1 and manually
          // checking the segment between the
          // last vertex and first one before this? Well
          val x1 = polygon.xpoints[i]
          val y1 = polygon.ypoints[i]
          val x2 = polygon.xpoints[j]
          val y2 = polygon.ypoints[j]
          if (Line2D.linesIntersect(x1, y1, x2, y2, edge.start.x, edge.start.y, edge.end.x, edge.end.y)) {
            return false
          }
        }
      }
      edges.add(edge)
      edge.start.addNeighbor(edge.end)
      edge.end.addNeighbor(edge.start)
      return true
    }
    return true
  }

  // Overide edges (WARNING DOESN'T CHECK FOR OBSTACLES)
  fun addEdge(edge: Edge) {
    edges.add(edge)
    edge.start.addNeighbor(edge.end)
    edge.end.addNeighbor(edge.start)
  }

  fun containsEdge(edge: Edge): Boolean {
    return edges.contains(edge)
  }

  // Find a path through the navigation mesh from the start node to the goal node
  fun findPath(start: Node?, goal: Node): List<Node?>? {
    // Use A* search to find the shortest path through the navigation mesh
    val closedSet: MutableSet<Node?> = HashSet()
    val openSet: MutableSet<Node?> = HashSet()
    val gScore: MutableMap<Node?, Double> = HashMap()
    val fScore: MutableMap<Node?, Double> = HashMap()
    val cameFrom: MutableMap<Node?, Node?> = HashMap()
    gScore[start] = 0.0
    fScore[start] = distance(start, goal)
    openSet.add(start)
    while (openSet.isNotEmpty()) {
      val current = getLowestFScore(openSet, fScore)
      if (current == goal) {
        return reconstructPath(cameFrom, current)
      }
      openSet.remove(current)
      closedSet.add(current)
      for (neighbor in current!!.neighbors) {
        if (!closedSet.contains(neighbor)) {
          val tentativeGScore = gScore[current]!! + distance(current, neighbor)
          if (!openSet.contains(neighbor) || tentativeGScore < gScore[neighbor]!!) {
            cameFrom[neighbor] = current
            gScore[neighbor] = tentativeGScore
            fScore[neighbor] = gScore[neighbor]!! + distance(neighbor, goal)
            // No check needed, .add is a noop if it contains it. Sets don't allow
            // duplicates
            openSet.add(neighbor)
          }
        }
      }
    }

    // If we get here, then no path was found
    return null
  }

  // Get the node in the open set with the lowest f score
  private fun getLowestFScore(openSet: Set<Node?>, fScore: Map<Node?, Double>): Node? {
    var lowestFScoreNode: Node? = null
    var lowestFScore = Double.MAX_VALUE
    for (node in openSet) {
      val f = fScore[node]!!
      if (f < lowestFScore) {
        lowestFScore = f
        lowestFScoreNode = node
      }
    }
    return lowestFScoreNode
  }

  // Reconstruct the path from the start node to the goal node
  private fun reconstructPath(cameFrom: Map<Node?, Node?>, current: Node?): List<Node?> {
    var cNode = current
    val path: MutableList<Node?> = ArrayList()
    path.add(cNode)
    while (cameFrom.containsKey(cNode)) {
      cNode = cameFrom[cNode]
      path.add(cNode)
    }
    path.reverse()
    return path
  }

  companion object {
    // Calculate the distance between two nodes
    private fun distance(n1: Node?, n2: Node): Double {
      val dx = n1!!.x - n2.x
      val dy = n1.y - n2.y
      return hypot(dx, dy)
    }
  }
}
