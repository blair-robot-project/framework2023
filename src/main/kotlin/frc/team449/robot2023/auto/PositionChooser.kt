package frc.team449.robot2023.auto

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser

class PositionChooser : SendableChooser<PositionChooser.Positions>() {
  enum class Positions {
    POSITION1,
    POSITION2
  }

  init {
    this.setDefaultOption("Position 1", Positions.POSITION1)
    this.addOption("Position 2", Positions.POSITION2)
  }
}
