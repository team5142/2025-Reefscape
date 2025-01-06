// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.DebugTelemetrySubsystems;
/**
 * Push note into the shooter, end when note leaves the intake
 */
public class ShootingPushNote extends Command {
  /** Creates a new ShootingPushNote. */
  private double power;
  public ShootingPushNote(double p) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.intakeSubsystem);
    power = p;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if ( DebugTelemetrySubsystems.intake) {
      System.out.println("--- Running intake with speed "+ power);
    }
    RobotContainer.intakeSubsystem.runIntake(power);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !RobotContainer.intakeSubsystem.isNoteInIntake();
  }
}
