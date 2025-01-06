// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.GPMConstants.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootingGPM0Sequence extends SequentialCommandGroup {
  /** Creates a new ShootingClose0. */
  public ShootingGPM0Sequence(double distance) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // Spin the shooter first
      new WaitCommand(1.5)
        .raceWith(
          // get shooter rollers up to speed
          new ShooterToPower(RobotContainer.gpmHelpers.getGPM0ShooterPower(distance))
            .alongWith(
              new ArmTurnToAngle(() -> RobotContainer.gpmHelpers.getGPM0Angle(distance))
            )
        ),
      // push note to shooter after shooter rollers get up to speed
      new ConditionalCommand(
        // command if the note is seen by the sensor at this point
        new WaitCommand(0.75)
          .raceWith(
              new ShootingPushNote( // push note by the intake to the shooter rollers
                RobotContainer.gpmHelpers.getGPM0IntakePower(distance)))
                // after the note leaves the intake, wait to make sure it leaves the shooter as well
                .andThen(new WaitCommand(Shooter.SHOOT_TIME_DELAY_AFTER_NOTE_LEAVES))
        ,
        // slower command if the note is not seen by the sensor at this point
        (new WaitCommand(1.5)
          .raceWith(new IntakeRun(
            RobotContainer.gpmHelpers.getGPM0IntakePower(distance)))
        )
        ,
        // is note seen by the sensor?
        RobotContainer.intakeSubsystem::isNoteInIntake),
      // wait until the shooting is done
      new PrintCommand("GPM0Sequence done")
      
    );
  }
}
