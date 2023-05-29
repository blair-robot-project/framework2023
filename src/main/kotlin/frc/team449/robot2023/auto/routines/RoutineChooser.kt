package frc.team449.robot2023.auto.routines

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.robot2023.Robot
import frc.team449.robot2023.auto.PositionChooser

class RoutineChooser(private val robot: Robot) : SendableChooser<String>() {

  fun routineMap(): HashMap<String, Command> {
    return hashMapOf(
      "RedExample1" to Example(robot, PositionChooser.Positions.POSITION1, true).createCommand(),
      "RedExample2" to Example(robot, PositionChooser.Positions.POSITION2, true).createCommand(),
      "BlueExample1" to Example(robot, PositionChooser.Positions.POSITION1, false).createCommand(),
      "BlueExample2" to Example(robot, PositionChooser.Positions.POSITION2, false).createCommand()
    )
  }

  init {
    updateOptions(PositionChooser.Positions.POSITION1, DriverStation.getAlliance() == DriverStation.Alliance.Red)
  }

  fun updateOptions(position: PositionChooser.Positions, isRed: Boolean) {
    /** Add auto options here */
    this.setDefaultOption("Do Nothing", "DropCone")

    this.addOption(
      "Example Auto",
      if (isRed) {
        when (position) {
          PositionChooser.Positions.POSITION1 -> {
            "RedExample1"
          } else -> {
            "RedExample2"
          }
        }
      } else {
        when (position) {
          PositionChooser.Positions.POSITION1 -> {
            "BlueExample1"
          } else -> {
            "BlueExample2"
          }
        }
      }
    )
  }
}
