// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.lib.TrajectoryHelpers;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunTrajectorySequenceRobotAtStartPoint extends SequentialCommandGroup {
  /** Creates a new RunTrajectorySequenceRobotAtStartPoint. */

  PathPlannerPath trajectoryPath;

  public RunTrajectorySequenceRobotAtStartPoint() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands();
  }

    /**
   * Creates a new RunTrajectorySequenceRobotAtStartPoint.
   * This command reads a PathPlanner trajectory from a file specified by name in
   * the first parameter,
   * optionally applies max velocity and max acceleration restrictions as well as
   * reverse if needed,
   * and runs the adjusted trajectory via AutonomousTrajectoryRioCommand command.
   */
  public RunTrajectorySequenceRobotAtStartPoint(String trajectoryName, double maxVelocity, double maxAngularVelocity, 
            double maxAcceleration, double maxAngularAcceleration) throws Exception {
        this(
          TrajectoryHelpers.replanTrajectory(
            PathPlannerPath.fromPathFile(trajectoryName),
            maxVelocity, maxAngularVelocity, maxAcceleration, maxAngularAcceleration)
            
        );
        System.out.println("initalized trajectory: "+ trajectoryName + "V:"+maxVelocity+" A:"+maxAngularVelocity);
    }

  public RunTrajectorySequenceRobotAtStartPoint(String trajectoryName) throws Exception{

    this(
          PathPlannerPath.fromPathFile(trajectoryName)
        );
        System.out.println("initalized trajectory: "+ trajectoryName);
  }

  public RunTrajectorySequenceRobotAtStartPoint(PathPlannerPath traj) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // Read the trajectory from a file
    this.trajectoryPath = traj;
    PathPlannerPath drivePath = (RobotContainer.isAllianceRed)?trajectoryPath.flipPath():trajectoryPath;

    RobotConfig robotConfig;

    try {
      robotConfig = RobotConfig.fromGUISettings();
      addCommands(
          // new InstantCommand(RobotContainer.driveSubsystem::zeroDriveEncoders),
          new PrintCommand("****Starting trajectory****"),
          // new WaitCommand(0.4),
          new InstantCommand(() -> RobotContainer.driveSubsystem
              .setYawForTrajectory(drivePath.getStartingHolonomicPose().orElse(new Pose2d()).getRotation().getDegrees())),
          new InstantCommand(() -> RobotContainer.driveSubsystem.resetOdometry(drivePath.getStartingHolonomicPose().orElse(new Pose2d()))),
          // new PrintCommand(
          // "START IX:" + trajectoryPath.getInitialPose().getX()+
          // " IY:" + trajectoryPath.getInitialPose().getY()+
          // " IA:" + trajectoryPath.getInitialPose().getRotation().getDegrees()
          // ), // Set the initial pose of the robot to the one in a trajectory
          new AutonomousTrajectoryRioCommand(drivePath, robotConfig) 
          //, // Run a trajectory
          .finallyDo (
            () -> RobotContainer.driveSubsystem.restoreYawAfterTrajectory()
          )
          ,
          new PrintCommand("****End trajectory****")
      );
    } catch (Exception e) {
      addCommands(
        new PrintCommand("****Could not read the RobotConfig from GUI settings****")
      );
    }
  }

}
