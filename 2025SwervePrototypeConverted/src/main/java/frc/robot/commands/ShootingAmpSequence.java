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
public class ShootingAmpSequence extends SequentialCommandGroup {
  /** Creates a new ShootingAmpSequence. */
  public ShootingAmpSequence() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // Spin the shooter and raise arm
      new WaitCommand(2.3).raceWith( // race wait with the shooter/arm commands
        // Shooter to power
        new ShooterToPower(RobotContainer.gpmHelpers.getShooterPowerTouchingAmp()).
          alongWith(
            // Arm to angle
            new ArmTurnToAngle(() -> RobotContainer.gpmHelpers.getAngleTouchingAmp())
          )
      ),
      // push note to shooter
      new IntakeRun(RobotContainer.gpmHelpers.getIntakePowerTouchingAmp()),
      // wait until the shooting is done
      new WaitCommand(1.5)
    );
  }
}
