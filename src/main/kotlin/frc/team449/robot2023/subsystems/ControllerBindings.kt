package frc.team449.robot2023.subsystems

import edu.wpi.first.wpilibj.XboxController
import frc.team449.robot2023.Robot

class ControllerBindings(
  private val driveController: XboxController,
  private val mechanismController: XboxController,
  private val robot: Robot
) {

  /**
   * Add controller bindings here.
   */
  fun bindButtons() {
    /** Example:
     * JoystickButton(driveController, XboxController.Button.kLeftBumper.value).onTrue(
     *       robot.endEffector.runOnce(robot.endEffector::intakeReverse).andThen(
     *         robot.groundIntake.outtake()
     *       )
     *     ).onFalse(
     *       robot.endEffector.runOnce(robot.endEffector::stop).andThen(
     *         robot.groundIntake.runOnce(robot.groundIntake::stop)
     *       )
     *     }
     */
  }
}
