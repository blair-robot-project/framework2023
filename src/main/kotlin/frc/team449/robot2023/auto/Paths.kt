package frc.team449.robot2023.auto

import com.pathplanner.lib.PathPlanner
import com.pathplanner.lib.PathPlannerTrajectory

object Paths {
  object WALL {
    val CUBE: MutableList<PathPlannerTrajectory> =
      PathPlanner.loadPathGroup(
        "wallCube",
        PathPlanner.getConstraintsFromPath("wallCube")
      )

    /** Description of path: Wall side path that scores a cone, then a cube, and then balances on the station */
    val CONECUBESTATION: MutableList<PathPlannerTrajectory> =
      PathPlanner.loadPathGroup(
        "wallConeCubeStation",
        PathPlanner.getConstraintsFromPath("wallConeCubeStation")
      )

    /** Description of path: Wall side path that scores a cone and then balances on the station */
    val CONESTATION: MutableList<PathPlannerTrajectory> =
      PathPlanner.loadPathGroup(
        "wallConeStation",
        PathPlanner.getConstraintsFromPath("wallConeStation")
      )

    /** Description of path: Wall side path that scores a cone, then a cube */
    val CONECUBE: MutableList<PathPlannerTrajectory> =
      PathPlanner.loadPathGroup(
        "bumpConeCube",
        PathPlanner.getConstraintsFromPath("bumpConeCube")
      )

    /** Description of path: Wall side path that scores a cone and aligns up to a game piece for teleop */
    val CONE: MutableList<PathPlannerTrajectory> =
      PathPlanner.loadPathGroup(
        "wallCone",
        PathPlanner.getConstraintsFromPath("wallCone")
      )

    val CUBESTATION: MutableList<PathPlannerTrajectory> =
      PathPlanner.loadPathGroup(
        "wallCubeStation",
        PathPlanner.getConstraintsFromPath("wallCubeStation")
      )

    val CUBECONESTATION: MutableList<PathPlannerTrajectory> =
      PathPlanner.loadPathGroup(
        "wallCubeConeStation",
        PathPlanner.getConstraintsFromPath("wallCubeConeStation")
      )

    val CUBECONE: MutableList<PathPlannerTrajectory> =
      PathPlanner.loadPathGroup(
        "wallCubeCone",
        PathPlanner.getConstraintsFromPath("wallCubeCone")
      )

    val CUBECONECONE: MutableList<PathPlannerTrajectory> =
      PathPlanner.loadPathGroup(
        "wallCubeConeCone",
        PathPlanner.getConstraintsFromPath("wallCubeConeCone")
      )

    val CONECUBECONE: MutableList<PathPlannerTrajectory> =
      PathPlanner.loadPathGroup(
        "wallConeCubeCone",
        PathPlanner.getConstraintsFromPath("wallConeCubeCone")
      )

    val CONECUBECUBE: MutableList<PathPlannerTrajectory> =
      PathPlanner.loadPathGroup(
        "wallConeCubeCube",
        PathPlanner.getConstraintsFromPath("wallConeCubeCube Copy")
      )
  }

  object FAR {
    val CUBE: MutableList<PathPlannerTrajectory> =
      AutoUtil.transformForFarSide(
        PathPlanner.loadPathGroup(
          "wallCube",
          PathPlanner.getConstraintsFromPath("wallCube")
        )
      )

    /** Description of path: Far (near center of field) side path that scores a cone, then a cube, and then balances on the station */
    val CONECUBESTATION: MutableList<PathPlannerTrajectory> =
      AutoUtil.transformForFarSide(
        PathPlanner.loadPathGroup(
          "wallConeCubeStation",
          PathPlanner.getConstraintsFromPath("wallConeCubeStation")
        )
      )

    /** Description of path: Far (near center of field) side path that scores a cone, and then balances on the station */
    val CONESTATION: MutableList<PathPlannerTrajectory> =
      AutoUtil.transformForFarSide(
        PathPlanner.loadPathGroup(
          "wallConeStation",
          PathPlanner.getConstraintsFromPath("wallConeStation")
        )
      )

    /** Description of path: Far (near center of field) side path that scores a cone, then a cube */
    val CONECUBE: MutableList<PathPlannerTrajectory> =
      AutoUtil.transformForFarSide(
        PathPlanner.loadPathGroup(
          "wallConeCube",
          PathPlanner.getConstraintsFromPath("wallConeCube")
        )
      )

    /** Description of path: Far (near center of field) side path that scores a cone, and aligns up to a game piece for teleop */
    val CONE: MutableList<PathPlannerTrajectory> =
      AutoUtil.transformForFarSide(
        PathPlanner.loadPathGroup(
          "wallCone",
          PathPlanner.getConstraintsFromPath("wallCone")
        )
      )

    val CUBESTATION: MutableList<PathPlannerTrajectory> =
      AutoUtil.transformForFarSide(
        PathPlanner.loadPathGroup(
          "wallCubeStation",
          PathPlanner.getConstraintsFromPath("wallCubeStation")
        )
      )

    val CUBECONESTATION: MutableList<PathPlannerTrajectory> =
      AutoUtil.transformForFarSide(
        PathPlanner.loadPathGroup(
          "wallCubeConeStation",
          PathPlanner.getConstraintsFromPath("wallCubeConeStation")
        )
      )

    val CUBECONE: MutableList<PathPlannerTrajectory> =
      AutoUtil.transformForFarSide(
        PathPlanner.loadPathGroup(
          "wallCubeCone",
          PathPlanner.getConstraintsFromPath("wallCubeCone")
        )
      )

    val CUBECONECONE: MutableList<PathPlannerTrajectory> =
      AutoUtil.transformForFarSide(
        PathPlanner.loadPathGroup(
          "wallCubeConeCone",
          PathPlanner.getConstraintsFromPath("wallCubeConeCone")
        )
      )

    val CONECUBECONE: MutableList<PathPlannerTrajectory> =
      AutoUtil.transformForFarSide(
        PathPlanner.loadPathGroup(
          "wallConeCubeCone",
          PathPlanner.getConstraintsFromPath("wallConeCubeCone")
        )
      )

    val CONECUBECUBE: MutableList<PathPlannerTrajectory> =
      AutoUtil.transformForFarSide(
        PathPlanner.loadPathGroup(
          "wallConeCubeCube Copy",
          PathPlanner.getConstraintsFromPath("wallConeCubeCube Copy")
        )
      )
  }

  object CENTER {
    val CUBEBALANCE: MutableList<PathPlannerTrajectory> =
      PathPlanner.loadPathGroup(
        "centerCubeBalance",
        PathPlanner.getConstraintsFromPath("centerCubeBalance")
      )
  }
}
