package frc.team449.robot2023.auto

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser

class PositionChooser : SendableChooser<PositionChooser.Positions>() {
  enum class Positions {
    FARCONE,
    WALLCONE,
    FARCUBE,
    WALLCUBE,
    CENTER
  }

  init {
    this.setDefaultOption("Cone Wall side", Positions.WALLCONE)
    this.addOption("Cone Far side", Positions.FARCONE)
    this.addOption("Cube Wall side", Positions.WALLCUBE)
    this.addOption("Cube Far side", Positions.FARCUBE)
    this.addOption("Center Cube (Only drop piece and 1 Piece Balance)", Positions.CENTER)
  }
}
