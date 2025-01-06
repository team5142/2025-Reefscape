// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.GPMConstants.Arm;
import frc.robot.RobotContainer;

public class CalibrateArmPositionForShooterCalibration extends Command {
  /** Creates a new CalibrateGpmShootingWithArm. */
  public CalibrateArmPositionForShooterCalibration() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("**** Hold Arm ...");
    // Z-trottle down = +1, so reverse
    double armRange = -RobotContainer.driveStick3.getRawAxis(OIConstants.CALIBRATION_JOYSTICK_SLIDER_AXLE);

    // Set angle as proportional between MIN and MAX angles, with the proportion identified by the throttle

    double angle = Arm.ARM_MIN_ANGLE + (Arm.ARM_MAX_ANGLE-Arm.ARM_MIN_ANGLE)*(armRange+1)/2.0;
    RobotContainer.armSubsystem.setArmMotorAnglesSI(angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("End of Arm PID movement");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
