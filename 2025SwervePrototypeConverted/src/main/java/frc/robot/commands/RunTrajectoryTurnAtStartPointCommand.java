// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunTrajectoryTurnAtStartPointCommand extends SequentialCommandGroup {
  /** Creates a new RunTrajectorySequenceRobotAtStartPoint. */

  PathPlannerPath trajectoryPath;

  public RunTrajectoryTurnAtStartPointCommand() {
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
  public RunTrajectoryTurnAtStartPointCommand(String trajectoryName, double maxVelocity, double maxAngularVelocity) {
        this(
            PathPlannerPath.fromPathFile(trajectoryName)
                .replan(
                    PathPlannerPath.fromPathFile(trajectoryName).getStartingDifferentialPose()
                    , new ChassisSpeeds(maxVelocity, maxVelocity, maxAngularVelocity)
                    )
        );
        System.out.println("initalized trajectory: "+ trajectoryName + "V:"+maxVelocity+" A:"+maxAngularVelocity);
    }

  public RunTrajectoryTurnAtStartPointCommand(String trajectoryName){
        this(
          PathPlannerPath.fromPathFile(trajectoryName)
        );
        System.out.println("initalized trajectory: "+ trajectoryName);
  }


  public RunTrajectoryTurnAtStartPointCommand(PathPlannerPath traj) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // Read the trajectory from a file
    this.trajectoryPath = traj;

    addCommands(
        // new InstantCommand(RobotContainer.driveSubsystem::zeroDriveEncoders),
        new PrintCommand("****Starting trajectory****"),
        // new WaitCommand(0.4),
        new InstantCommand(() -> RobotContainer.driveSubsystem
            .setYawForTrajectory(trajectoryPath.getPreviewStartingHolonomicPose().getRotation().getDegrees())),
        new InstantCommand(() -> RobotContainer.driveSubsystem.resetOdometry(trajectoryPath.getPreviewStartingHolonomicPose())),
        // new PrintCommand(
        // "START IX:" + trajectoryPath.getInitialPose().getX()+
        // " IY:" + trajectoryPath.getInitialPose().getY()+
        // " IA:" + trajectoryPath.getInitialPose().getRotation().getDegrees()
        // ), // Set the initial pose of the robot to the one in a trajectory
        new AutonomousTurnCommand(trajectoryPath)
        //, // Run a trajectory
        .finallyDo (
          () -> RobotContainer.driveSubsystem.restoreYawAfterTrajectory()
        )
        ,
        new PrintCommand("****End trajectory****"));
  }

}
