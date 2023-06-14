package frc.team449.control.obstacleAvoidance

class MapCreator {
  fun createGraph(aStarMap: VisGraph, obstacles: List<Obstacle>): VisGraph {
    aStarMap.addNode(Node(2.40 - 0.1, 4.75))
    aStarMap.addNode(Node(5.40 + 0.1, 4.75))
    aStarMap.addNode(Node(5.40 + 0.1, 0.75))
    aStarMap.addNode(Node(2.40 - 0.1, 0.75))
    // Divider
    // AStarMap.addNode(new Node(3.8 + 0.1, 4.75));
    for (i in 0 until aStarMap.nodeSize) {
      val startNode: Node = aStarMap.getNode(i)
      for (j in i + 1 until aStarMap.nodeSize) {
        aStarMap.addEdge(Edge(startNode, aStarMap.getNode(j)), obstacles)
      }
    }

    return aStarMap
  }
}
