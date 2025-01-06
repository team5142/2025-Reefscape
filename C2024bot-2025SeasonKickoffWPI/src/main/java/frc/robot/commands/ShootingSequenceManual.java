// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootingSequenceManual extends SequentialCommandGroup {
  /** Creates a new ShootingSequenceManual. */
  public ShootingSequenceManual() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ShooterToPower(RobotContainer.gpmHelpers.getGPM0ShooterPower(0))
            .raceWith(new WaitCommand(0.5)),
        new IntakeRun(RobotContainer.gpmHelpers.getGPM0ShooterPower(
            RobotContainer.gpmHelpers.getGPM0IntakePower(0))),
        // wait until the shooting is done
        new WaitCommand(1.5).raceWith(    
          new WaitUntilCommand( () -> ! RobotContainer.intakeSubsystem.isNoteInIntake() )
            .andThen(new WaitCommand(0.3))
        )
    );
  }
}
