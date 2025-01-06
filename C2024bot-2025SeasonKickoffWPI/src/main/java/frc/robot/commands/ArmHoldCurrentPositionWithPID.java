// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class ArmHoldCurrentPositionWithPID extends Command {

  double currentPosition; // will be lead encoder value

  /** Creates a new ArmHoldCurrentPositionWithPID. */
  public ArmHoldCurrentPositionWithPID() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentPosition = RobotContainer.armSubsystem.getArmEncoderLeader();

    // alex test
    //System.out.println("Hold position: "+currentPosition);

    RobotContainer.armSubsystem.stopArmMotors();
    RobotContainer.armSubsystem.setArmMotorEncoder(currentPosition);
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
    return false;
  }
}
