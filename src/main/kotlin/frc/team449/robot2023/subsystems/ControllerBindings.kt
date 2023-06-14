package frc.team449.robot2023.subsystems

import com.pathplanner.lib.PathConstraints
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import frc.team449.control.obstacleAvoidance.*
import frc.team449.robot2023.Robot

class ControllerBindings(
  private val driveController: XboxController,
  private val mechanismController: XboxController,
  private val robot: Robot
) {

  /** Sets the controls for the robot. */
  fun bindButtons() {
    val standardMap = VisGraph()

    MapCreator().createGraph(standardMap, FieldConstants.shortObstacles)

    JoystickButton(driveController, XboxController.Button.kX.value).onTrue(
      PPAStar(
        robot.drive,
        PathConstraints(4.0, 5.75),
        Node(Pose2d(2.0, 2.0, Rotation2d(0.0))),
        FieldConstants.shortObstacles,
        standardMap,
        robot.field
      )
    )

    /** Example:
     * JoystickButton(driveController, XboxController.Button.kLeftBumper.value).onTrue(
     *        PrintCommand(" hello! ")
     *     ).onFalse(
     *       robot.endEffector.runOnce(robot.endEffector::stop).andThen(
     *         robot.groundIntake.runOnce(robot.groundIntake::stop)
     *       )
     *     }
     */
  }
}
