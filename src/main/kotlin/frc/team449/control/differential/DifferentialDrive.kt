package frc.team449.control.differential

import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.control.DriveSubsystem
import frc.team449.robot2023.constants.RobotConstants
import frc.team449.robot2023.constants.drives.DifferentialConstants
import frc.team449.system.AHRS
import frc.team449.system.encoder.QuadEncoder
import frc.team449.system.motor.WrappedMotor
import frc.team449.system.motor.createSparkMax
import io.github.oblarg.oblog.Loggable
import io.github.oblarg.oblog.annotations.Log

/**
 * A differential drive with closed-loop velocity control using PID
 * @param makeVelPID Used to make two copies of the same PIDController for both sides of the
 * drivetrain
 */
open class DifferentialDrive(
  private val leftLeader: WrappedMotor,
  private val rightLeader: WrappedMotor,
  private val ahrs: AHRS,
  private val feedforward: SimpleMotorFeedforward,
  private val makeVelPID: () -> PIDController,
  private val trackwidth: Double
) : DriveSubsystem, SubsystemBase(), Loggable {
  init {
    leftLeader.encoder.resetPosition(0.0)
    rightLeader.encoder.resetPosition(0.0)
  }

  /**
   * The kinematics used to convert [DifferentialDriveWheelSpeeds] to [ChassisSpeeds]
   */
  val kinematics = DifferentialDriveKinematics(trackwidth)

  /** Odometer to keep track of where the robot is */
  val odometer = DifferentialDriveOdometry(ahrs.heading, leftLeader.position, rightLeader.position)

  /** Velocity PID controller for left side */
  val leftPID = makeVelPID()

  /** Velocity PID controller for right side */
  val rightPID = makeVelPID()

  /** Variable to keep track of the wheel speeds*/
  @Log.ToString(name = "Desired Differential Speeds")
  var wheelSpeeds = DifferentialDriveWheelSpeeds(0.0, 0.0)

  private var previousTime = Timer.getFPGATimestamp()
  private var prevWheelSpeeds = DifferentialDriveWheelSpeeds(0.0, 0.0)

  init {
    leftLeader.encoder.resetPosition(0.0)
    rightLeader.encoder.resetPosition(0.0)
  }

  /**
   * Convert from x, y, rotation to left and right speeds
   *
   * @param desiredSpeeds The [ChassisSpeeds] desired for the drive
   */

  override fun set(desiredSpeeds: ChassisSpeeds) {
    prevWheelSpeeds = wheelSpeeds
    wheelSpeeds = kinematics.toWheelSpeeds(desiredSpeeds)
    wheelSpeeds.desaturate(RobotConstants.MAX_LINEAR_SPEED)
  }

  @get:Log.ToString(name = "Pose")
  override var pose: Pose2d
    get() = this.odometer.poseMeters
    set(pose) {
      leftLeader.encoder.resetPosition(0.0)
      rightLeader.encoder.resetPosition(0.0)
      this.odometer.resetPosition(ahrs.heading, leftLeader.position, rightLeader.position, pose)
    }

  override fun stop() {
    set(ChassisSpeeds(.0, .0, .0))
  }

  /** Periodically update the odometer */
  override fun periodic() {
    val currentTime = Timer.getFPGATimestamp()

    val dt = if (!previousTime.isNaN()) {
      currentTime - previousTime
    } else {
      0.02
    }
    val leftVel = wheelSpeeds.leftMetersPerSecond
    val rightVel = wheelSpeeds.rightMetersPerSecond
    val prevLeftVel = prevWheelSpeeds.leftMetersPerSecond
    val prevRightVel = prevWheelSpeeds.rightMetersPerSecond

    leftLeader.setVoltage(
      feedforward.calculate(prevLeftVel, leftVel, dt) + leftPID.calculate(leftLeader.velocity, leftVel)
    )
    rightLeader.setVoltage(
      feedforward.calculate(prevRightVel, rightVel, dt) + rightPID.calculate(rightLeader.velocity, rightVel)
    )

    this.odometer.update(ahrs.heading, this.leftLeader.position, this.rightLeader.position)

    previousTime = currentTime
  }

  companion object {
    /** Helper to make each side for the differential drive */
    private fun makeSide(
      name: String,
      motorId: Int,
      inverted: Boolean,
      encInverted: Boolean,
      wpiEnc: edu.wpi.first.wpilibj.Encoder,
      followers: Map<Int, Boolean>
    ) =
      createSparkMax(
        name = name + "_Drive",
        id = motorId,
        enableBrakeMode = true,
        inverted = inverted,
        encCreator = QuadEncoder.creator(
          wpiEnc,
          DifferentialConstants.DRIVE_EXT_ENC_CPR,
          DifferentialConstants.DRIVE_UPR,
          DifferentialConstants.DRIVE_GEARING,
          encInverted
        ),

        slaveSparks = followers,
        currentLimit = DifferentialConstants.DRIVE_CURRENT_LIM
      )

    fun createDifferentialDrive(ahrs: AHRS): DifferentialDrive {
      return DifferentialDrive(
        leftLeader = makeSide(
          "Left",
          DifferentialConstants.DRIVE_MOTOR_L,
          inverted = false,
          encInverted = false,
          wpiEnc = DifferentialConstants.DRIVE_ENC_LEFT,
          followers = mapOf(
            DifferentialConstants.DRIVE_MOTOR_L1 to false,
            DifferentialConstants.DRIVE_MOTOR_L2 to false
          )
        ),
        rightLeader = makeSide(
          "Right",
          DifferentialConstants.DRIVE_MOTOR_R,
          inverted = true,
          encInverted = true,
          wpiEnc = DifferentialConstants.DRIVE_ENC_RIGHT,
          followers = mapOf(
            DifferentialConstants.DRIVE_MOTOR_R1 to false,
            DifferentialConstants.DRIVE_MOTOR_R2 to false
          )
        ),
        ahrs,
        SimpleMotorFeedforward(
          DifferentialConstants.DRIVE_FF_KS,
          DifferentialConstants.DRIVE_FF_KV,
          DifferentialConstants.DRIVE_FF_KA
        ),
        {
          PIDController(
            DifferentialConstants.DRIVE_KP_VEL,
            DifferentialConstants.DRIVE_KI_VEL,
            DifferentialConstants.DRIVE_KD_VEL
          )
        },
        DifferentialConstants.TRACK_WIDTH
      )
    }

    fun simOf(
      drive: DifferentialDrive,
      kV: Double,
      kA: Double,
      angleKV: Double,
      angleKA: Double,
      wheelRadius: Double
    ): DifferentialSim {
      val drivePlant = LinearSystemId.identifyDrivetrainSystem(
        kV,
        kA,
        angleKV,
        angleKA
      )
      val driveSim = DifferentialDrivetrainSim(
        drivePlant,
        DCMotor.getNEO(3),
        DifferentialConstants.DRIVE_GEARING,
        drive.trackwidth,
        wheelRadius,
        VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)
      )
      return DifferentialSim(driveSim, drive.leftLeader, drive.rightLeader, drive.ahrs, drive.feedforward, drive.makeVelPID, drive.trackwidth)
    }
  }
}
