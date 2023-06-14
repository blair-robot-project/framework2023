package frc.team449.robot2023.auto.routines

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.robot2023.Robot
import frc.team449.robot2023.auto.PositionChooser

class RoutineChooser(private val robot: Robot) : SendableChooser<String>() {

  fun routineMap(): HashMap<String, Command> {
    return hashMapOf(
      "RedFarConeCube" to EdgeConeCube(robot, PositionChooser.Positions.FARCONE, true).createCommand(robot),
      "RedWallConeCube" to EdgeConeCube(robot, PositionChooser.Positions.WALLCONE, true).createCommand(robot),
      "RedFarCubeCone" to EdgeCubeCone(robot, PositionChooser.Positions.FARCUBE, true).createCommand(robot),
      "RedWallCubeCone" to EdgeCubeCone(robot, PositionChooser.Positions.WALLCUBE, true).createCommand(robot),
      "RedCenterCubeBalance" to CenterCubeStation(robot, true).createCommand(robot),
      "RedFarConeCubeCube" to EdgeConeCubeCube(robot, PositionChooser.Positions.FARCONE, true).createCommand(robot),
      "RedWallConeCubeCube" to EdgeConeCubeCube(robot, PositionChooser.Positions.WALLCONE, true).createCommand(robot),
      "BlueFarConeCube" to EdgeConeCube(robot, PositionChooser.Positions.FARCONE, false).createCommand(robot),
      "BlueWallConeCube" to EdgeConeCube(robot, PositionChooser.Positions.WALLCONE, false).createCommand(robot),
      "BlueFarCubeCone" to EdgeCubeCone(robot, PositionChooser.Positions.FARCUBE, false).createCommand(robot),
      "BlueWallCubeCone" to EdgeCubeCone(robot, PositionChooser.Positions.WALLCUBE, false).createCommand(robot),
      "BlueCenterCubeBalance" to CenterCubeStation(robot, false).createCommand(robot),
      "BlueFarConeCubeCube" to EdgeConeCubeCube(robot, PositionChooser.Positions.FARCONE, false).createCommand(robot),
      "BlueWallConeCubeCube" to EdgeConeCubeCube(robot, PositionChooser.Positions.WALLCONE, false).createCommand(robot),
      "DropCone" to DropCone(robot).createCommand(robot)
    )
  }

  init {
    updateOptions(PositionChooser.Positions.CENTER, DriverStation.getAlliance() == DriverStation.Alliance.Red)
  }

  fun updateOptions(position: PositionChooser.Positions, isRed: Boolean) {
    /** Add auto options here */
    this.setDefaultOption("Drop Piece", "DropCone")

    this.addOption(
      "2 Piece (Edges only)",
      if (isRed) {
        when (position) {
          PositionChooser.Positions.FARCONE -> {
            "RedFarConeCube"
          }

          PositionChooser.Positions.WALLCONE -> {
            "RedWallConeCube"
          }

          PositionChooser.Positions.FARCUBE -> {
            "RedFarCubeCone"
          }

          PositionChooser.Positions.WALLCUBE -> {
            "RedWallCubeCone"
          }

          else -> {
            "DropCone"
          }
        }
      } else {
        when (position) {
          PositionChooser.Positions.FARCONE -> {
            "BlueFarConeCube"
          }

          PositionChooser.Positions.WALLCONE -> {
            "BlueWallConeCube"
          }

          PositionChooser.Positions.FARCUBE -> {
            "BlueFarCubeCone"
          }

          PositionChooser.Positions.WALLCUBE -> {
            "BlueWallCubeCone"
          }

          else -> {
            "DropCone"
          }
        }
      }
    )

    this.addOption(
      "1 Piece and Balance",
      if (isRed) {
        when (position) {
          PositionChooser.Positions.CENTER -> {
            "RedCenterCubeBalance"
          }

          else -> {
            "DropCone"
          }
        }
      } else {
        when (position) {
          PositionChooser.Positions.CENTER -> {
            "BlueCenterCubeBalance"
          }

          else -> {
            "DropCone"
          }
        }
      }
    )

    this.addOption(
      "3 Piece (Starting Edge Cone Only)",
      if (isRed) {
        when (position) {
          PositionChooser.Positions.FARCONE -> {
            "RedFarConeCubeCube"
          }

          PositionChooser.Positions.WALLCONE -> {
            "RedWallConeCubeCube"
          }

          else -> {
            "DropCone"
          }
        }
      } else {
        when (position) {
          PositionChooser.Positions.FARCONE -> {
            "BlueFarConeCubeCube"
          }

          PositionChooser.Positions.WALLCONE -> {
            "BlueWallConeCubeCube"
          }

          else -> {
            "DropCone"
          }
        }
      }
    )
  }
}
