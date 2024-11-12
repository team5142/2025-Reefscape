// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.RobotContainer;

public class CalibrateShooterPower extends Command {
  /** Creates a new CalibrateShooterPower. */
  public CalibrateShooterPower() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("**** Calibrating Shooter ...");
    double shooterPower = RobotContainer.driveStick1.getRawAxis(OIConstants.CALIBRATION_JOYSTICK_SLIDER_AXLE);
    RobotContainer.shooterSubsystem.runShooterWithPower(shooterPower);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double shooterPower = RobotContainer.driveStick1.getRawAxis(OIConstants.CALIBRATION_JOYSTICK_SLIDER_AXLE);
    RobotContainer.shooterSubsystem.runShooterWithPower(shooterPower);
    SmartDashboard.putNumber("Calibration - Shooter Power", shooterPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.shooterSubsystem.runShooterWithPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
