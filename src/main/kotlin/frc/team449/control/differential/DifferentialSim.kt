package frc.team449.control.differential

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim
import frc.team449.system.AHRS
import frc.team449.system.encoder.Encoder
import frc.team449.system.motor.WrappedMotor

class DifferentialSim(
  private val driveSim: DifferentialDrivetrainSim,
  leftLeader: WrappedMotor,
  rightLeader: WrappedMotor,
  ahrs: AHRS,
  private val feedforward: SimpleMotorFeedforward,
  makeVelPID: () -> PIDController,
  trackwidth: Double
) : DifferentialDrive(leftLeader, rightLeader, ahrs, feedforward, makeVelPID, trackwidth) {

  private val leftEncSim = Encoder.SimController(leftLeader.encoder)
  private val rightEncSim = Encoder.SimController(rightLeader.encoder)

  private var offset: Rotation2d = Rotation2d()
  private var lastTime = Timer.getFPGATimestamp()

  override fun periodic() {
    val currTime = Timer.getFPGATimestamp()

    val dt = currTime - lastTime

    val leftVel = wheelSpeeds.leftMetersPerSecond
    val rightVel = wheelSpeeds.rightMetersPerSecond

    val leftVoltage = feedforward.calculate(leftVel) + leftPID.calculate(driveSim.leftVelocityMetersPerSecond, leftVel)
    val rightVoltage = feedforward.calculate(rightVel) + rightPID.calculate(driveSim.rightVelocityMetersPerSecond, rightVel)

    leftEncSim.velocity = driveSim.leftVelocityMetersPerSecond
    rightEncSim.velocity = driveSim.rightVelocityMetersPerSecond
    leftEncSim.position = driveSim.leftPositionMeters
    rightEncSim.position = driveSim.rightPositionMeters

    driveSim.setInputs(leftVoltage, rightVoltage)

    driveSim.update(dt)
    this.odometer.update(heading, this.leftEncSim.position, this.rightEncSim.position)
    this.lastTime = currTime
  }

  override var pose: Pose2d
    get() = driveSim.pose
    set(value) {
      driveSim.pose = value
    }

  override var heading: Rotation2d
    get() = driveSim.heading.rotateBy(offset)
    set(value) {
      offset = value - driveSim.heading
    }
}
