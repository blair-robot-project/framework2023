package frc.team449.robot2023.subsystems.light

import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.robot2023.constants.subsystem.LightConstants

class Light(
  port: Int,
  length: Int
) : SubsystemBase() {

  private val lightStrip = AddressableLED(port)
  val buffer = AddressableLEDBuffer(length)

  init {
    lightStrip.setLength(buffer.length)
    lightStrip.setData(buffer)
    lightStrip.start()
  }

  override fun periodic() {
    lightStrip.setData(buffer)
  }

  companion object {
    fun createLight(): Light {
      return Light(
        LightConstants.LIGHT_PORT,
        LightConstants.LIGHT_LENGTH
      )
    }
  }
}
