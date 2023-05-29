package frc.team449.control.holonomic

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.RobotBase.isReal
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.control.VisionEstimator
import frc.team449.robot2023.constants.RobotConstants
import frc.team449.robot2023.constants.drives.SwerveConstants
import frc.team449.robot2023.constants.vision.VisionConstants
import frc.team449.system.AHRS
import frc.team449.system.encoder.AbsoluteEncoder
import frc.team449.system.encoder.NEOEncoder
import frc.team449.system.motor.createSparkMax
import io.github.oblarg.oblog.annotations.Log
import kotlin.math.pow
import kotlin.math.sqrt

/**
 * @param modules the list of swerve modules on this drivetrain
 * @param ahrs the gyro that is mounted on the chassis
 * @param maxLinearSpeed the maximum translation speed of the chassis.
 * @param maxRotSpeed the maximum rotation speed of the chassis
 */
open class SwerveDrive(
  private val modules: List<SwerveModule>,
  private val ahrs: AHRS,
  override var maxLinearSpeed: Double,
  override var maxRotSpeed: Double,
  private val cameras: List<VisionEstimator> = mutableListOf()
) : SubsystemBase(), HolonomicDrive {

  private val kinematics = SwerveDriveKinematics(
    *this.modules
      .map { it.location }.toTypedArray()
  )

  @Log.ToString(name = "Current Speeds")
  var currentSpeeds = ChassisSpeeds()

  private val poseEstimator = SwerveDrivePoseEstimator(
    kinematics,
    ahrs.heading,
    getPositions(),
    RobotConstants.INITIAL_POSE,
    VisionConstants.ENCODER_TRUST,
    VisionConstants.VISION_TRUST
  )

  private var lastTime = Timer.getFPGATimestamp()

  @Log.ToString(name = "Desired Speeds")
  var desiredSpeeds: ChassisSpeeds = ChassisSpeeds()

  override fun set(desiredSpeeds: ChassisSpeeds) {
    this.desiredSpeeds = desiredSpeeds
  }

  val pitch: Rotation2d
    get() = Rotation2d(MathUtil.angleModulus(ahrs.pitch.radians))
  val roll: Rotation2d
    get() = Rotation2d(MathUtil.angleModulus(ahrs.roll.radians))

  /** The x y theta location of the robot on the field */
  override var pose: Pose2d
    @Log.ToString(name = "Pose")
    get() {
      return this.poseEstimator.estimatedPosition
    }
    set(value) {
      this.poseEstimator.resetPosition(
        ahrs.heading,
        getPositions(),
        value
      )
    }

  override fun stop() {
    this.set(ChassisSpeeds(0.0, 0.0, 0.0))
  }

  override fun periodic() {
    val currTime = Timer.getFPGATimestamp()

    currentSpeeds = kinematics.toChassisSpeeds(
      modules[0].state,
      modules[1].state,
      modules[2].state,
      modules[3].state
    )

    val desiredModuleStates =
      this.kinematics.toSwerveModuleStates(this.desiredSpeeds)

    /** If any module is going faster than the max speed,
     *  apply scaling down and make sure there isn't
     *  any early desaturation */
    SwerveDriveKinematics.desaturateWheelSpeeds(
      desiredModuleStates,
      SwerveConstants.MAX_ATTAINABLE_MK4I_SPEED
    )

    for (i in this.modules.indices) {
      this.modules[i].state = desiredModuleStates[i]
    }

    for (module in modules)
      module.update()

    // TODO: Test enabling vision
//    if (cameras.isNotEmpty()) localize()

    this.poseEstimator.update(
      ahrs.heading,
      getPositions()
    )

    this.lastTime = currTime
  }

  /**
   * @return an array of [SwerveModulePosition] for each module, containing [angle, position]
   */
  private fun getPositions(): Array<SwerveModulePosition> {
    return Array(modules.size) { i -> modules[i].position }
  }

  /**
   * @return an array of [SwerveModuleState] for each module, containing [angle, velocity]
   */
  private fun getStates(): Array<SwerveModuleState> {
    return Array(modules.size) { i -> modules[i].state }
  }

  private fun localize() = try {
    for (camera in cameras) {
      val result = camera.estimatedPose(pose.rotation)
      if (result.isPresent) {
        val presentResult = result.get()
        val numTargets = presentResult.targetsUsed.size
        var tagDistance = 0.0
        var avgAmbiguity = 0.0

        for (tag in presentResult.targetsUsed) {
          val tagPose = camera.fieldTags.getTagPose(tag.fiducialId)
          if (tagPose.isPresent) {
            val estimatedToTag = presentResult.estimatedPose.minus(tagPose.get())
            tagDistance += sqrt(estimatedToTag.x.pow(2) + estimatedToTag.y.pow(2)) / numTargets
            avgAmbiguity = tag.poseAmbiguity / numTargets
          } else {
            tagDistance = Double.MAX_VALUE
            avgAmbiguity = Double.MAX_VALUE
            break
          }
        }

        if (presentResult.timestampSeconds > 0 &&
          avgAmbiguity <= VisionConstants.MAX_AMBIGUITY &&
          numTargets < 2 && tagDistance <= VisionConstants.MAX_DISTANCE_SINGLE_TAG ||
          numTargets >= 2 && tagDistance <= VisionConstants.MAX_DISTANCE_MULTI_TAG
        ) {
          poseEstimator.addVisionMeasurement(
            presentResult.estimatedPose.toPose2d(),
            presentResult.timestampSeconds
          )
        }
      }
    }
  } catch (e: Exception) {
    print("!!!!!!!!! VISION ERROR !!!!!!! \n $e")
  }

  companion object {
    /** Create a swerve drivetrain using DriveConstants */
    fun createSwerve(ahrs: AHRS): SwerveDrive {
      val driveMotorController = { PIDController(SwerveConstants.DRIVE_KP, SwerveConstants.DRIVE_KI, SwerveConstants.DRIVE_KD) }
      val turnMotorController = { PIDController(SwerveConstants.TURN_KP, SwerveConstants.TURN_KI, SwerveConstants.TURN_KD) }
      val driveFeedforward = SimpleMotorFeedforward(SwerveConstants.DRIVE_KS, SwerveConstants.DRIVE_KV, SwerveConstants.DRIVE_KA)
      val modules = listOf(
        SwerveModule.create(
          "FLModule",
          makeDrivingMotor(
            "FL",
            SwerveConstants.DRIVE_MOTOR_FL,
            inverted = false
          ),
          makeTurningMotor(
            "FL",
            SwerveConstants.TURN_MOTOR_FL,
            inverted = true,
            sensorPhase = false,
            SwerveConstants.TURN_ENC_CHAN_FL,
            SwerveConstants.TURN_ENC_OFFSET_FL
          ),
          driveMotorController(),
          turnMotorController(),
          driveFeedforward,
          Translation2d(SwerveConstants.WHEELBASE / 2, SwerveConstants.TRACKWIDTH / 2)
        ),
        SwerveModule.create(
          "FRModule",
          makeDrivingMotor(
            "FR",
            SwerveConstants.DRIVE_MOTOR_FR,
            inverted = false
          ),
          makeTurningMotor(
            "FR",
            SwerveConstants.TURN_MOTOR_FR,
            inverted = true,
            sensorPhase = false,
            SwerveConstants.TURN_ENC_CHAN_FR,
            SwerveConstants.TURN_ENC_OFFSET_FR
          ),
          driveMotorController(),
          turnMotorController(),
          driveFeedforward,
          Translation2d(SwerveConstants.WHEELBASE / 2, -SwerveConstants.TRACKWIDTH / 2)
        ),
        SwerveModule.create(
          "BLModule",
          makeDrivingMotor(
            "BL",
            SwerveConstants.DRIVE_MOTOR_BL,
            inverted = false
          ),
          makeTurningMotor(
            "BL",
            SwerveConstants.TURN_MOTOR_BL,
            inverted = true,
            sensorPhase = false,
            SwerveConstants.TURN_ENC_CHAN_BL,
            SwerveConstants.TURN_ENC_OFFSET_BL
          ),
          driveMotorController(),
          turnMotorController(),
          driveFeedforward,
          Translation2d(-SwerveConstants.WHEELBASE / 2, SwerveConstants.TRACKWIDTH / 2)
        ),
        SwerveModule.create(
          "BRModule",
          makeDrivingMotor(
            "BR",
            SwerveConstants.DRIVE_MOTOR_BR,
            inverted = false
          ),
          makeTurningMotor(
            "BR",
            SwerveConstants.TURN_MOTOR_BR,
            inverted = true,
            sensorPhase = false,
            SwerveConstants.TURN_ENC_CHAN_BR,
            SwerveConstants.TURN_ENC_OFFSET_BR
          ),
          driveMotorController(),
          turnMotorController(),
          driveFeedforward,
          Translation2d(-SwerveConstants.WHEELBASE / 2, -SwerveConstants.TRACKWIDTH / 2)
        )
      )
      return if (isReal()) {
        SwerveDrive(
          modules,
          ahrs,
          RobotConstants.MAX_LINEAR_SPEED,
          RobotConstants.MAX_ROT_SPEED,
          VisionConstants.ESTIMATORS
        )
      } else {
        SwerveSim(
          modules,
          ahrs,
          RobotConstants.MAX_LINEAR_SPEED,
          RobotConstants.MAX_ROT_SPEED,
          VisionConstants.ESTIMATORS
        )
      }
    }

    /** Helper to make turning motors for swerve */
    private fun makeDrivingMotor(
      name: String,
      motorId: Int,
      inverted: Boolean
    ) =
      createSparkMax(
        name = name + "Drive",
        id = motorId,
        enableBrakeMode = true,
        inverted = inverted,
        encCreator =
        NEOEncoder.creator(
          SwerveConstants.DRIVE_UPR,
          SwerveConstants.DRIVE_GEARING
        ),
        currentLimit = SwerveConstants.DRIVE_CURRENT_LIM
      )

    /** Helper to make turning motors for swerve */
    private fun makeTurningMotor(
      name: String,
      motorId: Int,
      inverted: Boolean,
      sensorPhase: Boolean,
      encoderChannel: Int,
      offset: Double
    ) =
      createSparkMax(
        name = name + "Turn",
        id = motorId,
        enableBrakeMode = false,
        inverted = inverted,
        encCreator = AbsoluteEncoder.creator(
          encoderChannel,
          offset,
          SwerveConstants.TURN_UPR,
          sensorPhase
        ),
        currentLimit = SwerveConstants.STEERING_CURRENT_LIM
      )
  }
}
