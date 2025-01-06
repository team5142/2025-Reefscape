// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class NotePickupCamera extends SequentialCommandGroup {
  /** Creates a new NotePickupCamera. */
  public NotePickupCamera() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ArmDownToNoteVisionForAutoNotePickup(),
      new DeferredCommand(
        ()-> new InstantCommand(RobotContainer.photonVisionNoteHuntingSubsystem::xyAngleToNoteSaved)
              .andThen(new PrintCommand("P-A:"
                +RobotContainer.photonVisionNoteHuntingSubsystem.getxAngleToNoteSaved()
                +","+RobotContainer.photonVisionNoteHuntingSubsystem.getyAngleToNoteSaved()
                ))
        , Set.of()),

      new DeferredCommand(
        ()-> new ConditionalCommand(
          // new PrintCommand("Turn Parameters: " + 
          //   RobotContainer.photonVisionNoteHuntingSubsystem.getxAngleToNoteSaved() + " " + 
          //   RobotContainer.photonVisionNoteHuntingSubsystem.fromCameraToTargetSaved() + " " + 
          //   RobotContainer.photonVisionNoteHuntingSubsystem.angleToTurnToNote(
          //     RobotContainer.photonVisionNoteHuntingSubsystem.getxAngleToNoteSaved(),
          //     RobotContainer.photonVisionNoteHuntingSubsystem.fromCameraToTargetSaved()) 
          // )

          new TurnToRelativeAngleTrapezoidProfile(
              RobotContainer.photonVisionNoteHuntingSubsystem.angleToTurnToNote(
                    RobotContainer.photonVisionNoteHuntingSubsystem.getxAngleToNoteSaved(),
                    RobotContainer.photonVisionNoteHuntingSubsystem.fromCameraToTargetSaved()
              ), 
              ()->RobotContainer.driveSubsystem.getYaw())
              .andThen(
                new ParallelDeadlineGroup(
                  new PrintCommand("Driving for "+ RobotContainer.photonVisionNoteHuntingSubsystem.fromCameraToTargetSaved())
                    .andThen(new WaitCommand(3.0)),
                  new AutonomousTrajectory2Poses(
                    new Pose2d(),
                    new Pose2d(
                      RobotContainer.photonVisionNoteHuntingSubsystem.fromCameraToTargetSaved(),
                      0,
                      new Rotation2d()
                    ),
                    2.5,
                    1.5
                  ),
                  new ArmDownToIntake(),
                  new IntakeGrabNote()
                )
              )
              , 
          new PrintCommand("Cannot see note"), 
          ()-> !Double.isNaN(RobotContainer.photonVisionNoteHuntingSubsystem.getxAngleToNoteSaved())
        ),
        Set.of()
        )
      ,
    //cleanup
    new StopRobot(), // stop trajectory
    new ShooterStop(), // stop shooter
    new IntakeStop() // stop intake
  
    );
  }
}
