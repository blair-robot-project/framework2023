package frc.team449.robot2023.subsystems.arm.control

import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj.motorcontrol.MotorController
import frc.team449.system.encoder.AbsoluteEncoder
import frc.team449.system.encoder.EncoderCreator
import kotlin.math.PI

class ArmEncoder(
  name: String,
  encoder: DutyCycleEncoder,
  inverted: Boolean,
  offset: Double
) : AbsoluteEncoder(name, encoder, 2 * PI, inverted, offset) {

  override fun getPositionNative(): Double {
    return MathUtil.inputModulus(super.getPositionNative(), -0.5, 0.5)
  }

  companion object {
    /**
     * @param <T>
     * @param channel The DutyCycleEncoder port
     * @param offset The position to put into DutyCycleEncoder's setPositionOffset
     * @param inverted If the encoder needs to be inverted or not
     */
    fun <T : MotorController> creator(
      channel: Int,
      offset: Double,
      inverted: Boolean
    ): EncoderCreator<T> =
      EncoderCreator { name, _, _ ->
        val enc = ArmEncoder(
          name,
          DutyCycleEncoder(channel),
          inverted,
          offset
        )
        enc
      }
  }
}
