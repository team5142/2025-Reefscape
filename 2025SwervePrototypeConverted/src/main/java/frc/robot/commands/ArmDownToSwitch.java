// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GPMConstants.Arm;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ArmDownToSwitch extends Command {
  /** Creates a new ArmDownToSwitch. */
  public ArmDownToSwitch() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.armSubsystem.runArmMotors(Arm.armDownPowerForRecalibration);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.armSubsystem.stopArmMotors();
    System.out.println("ArmToSwitch I:"+interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.intakeSubsystem.isIntakeDown();
  }
}
