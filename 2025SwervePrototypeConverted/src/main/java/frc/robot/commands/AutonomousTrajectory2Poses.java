// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SwerveConstants.SwerveChassis;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
 /**
 * This command drives a robot between two Poses. Note that poses are provided as
 * Pose2d objects and not as suppliers. Since the command sequence is formed in a constructor,
 * you cannot use this class directly to drive between two poses with unknown values.
 * However, this Command is scheduled by AutonomousTrajectory2PosesDynamic Command,
 * which works with dynamically determined poses.
 */
public class AutonomousTrajectory2Poses extends SequentialCommandGroup {
  /**
   * Run a trajectory between 2 poses with max velocity and acceleration
   * @param startPose
   * @param endPose
   */
  public AutonomousTrajectory2Poses( Pose2d startPose, Pose2d endPose) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this(startPose, endPose, SwerveChassis.MaxSpeed, SwerveChassis.MaxAngularRate);
  }

  /**
   * Run trajectory between two defined poses with specified max velocity and acceleration
   * @param startPose
   * @param endPose
   * @param maxVelocity
   * @param maxAcceleration
   */
  public AutonomousTrajectory2Poses( Pose2d startPose, Pose2d endPose, double maxVelocity, double maxAcceleration) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    //System.out.println("******* Start 2 Pose Trajectory ");
    //System.out.println(startPose.toString());
    //System.out.println(endPose.toString());

    // Making sure not to drive non-existing poses; can happen with camera-aided navigation
    if (startPose ==  null || endPose ==  null) {
      return;
    }

    addCommands(
      new RunTrajectorySequenceRobotAtStartPoint(
        new PathPlannerPath(
          PathPlannerPath.bezierFromPoses(startPose, endPose),
          new PathConstraints(maxVelocity, maxAcceleration, SwerveChassis.MaxAngularRate, SwerveChassis.maxAngularAcceleration),
          new GoalEndState(0.0, endPose.getRotation())
        )
      )
    );
  }
}