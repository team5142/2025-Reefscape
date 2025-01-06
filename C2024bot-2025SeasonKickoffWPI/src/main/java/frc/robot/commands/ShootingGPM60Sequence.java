// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootingGPM60Sequence extends SequentialCommandGroup {
  /** Creates a new ShootingGPM60Sequence. */

  public ShootingGPM60Sequence(double distance) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
       // Spin the shooter first
      new WaitCommand(0.9)
        .raceWith(
          new ShooterToPower(RobotContainer.gpmHelpers.getGPM60ShooterPower(distance))
           .alongWith(
            // Along with arm to angle
              new ArmTurnToAngle(() -> RobotContainer.gpmHelpers.getGPM60Angle(distance))
           )
        ),
      // push note to shooter
      new IntakeRun(
          RobotContainer.gpmHelpers.getGPM60IntakePower(distance)),
      // wait until the shooting is done
      new WaitCommand(1.5)
    );
  }
}
