// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class SmartDashboardSubsystem extends SubsystemBase {
  /** Creates a new SmartDashboardSubsystem. */
  public SmartDashboardSubsystem() {}

  public void updateOdometryTelemetry() {
    for (int i =0; i<4; i++){
      SmartDashboard.putNumber("S"+i+" Drive Encoder", RobotContainer.driveSubsystem.getDriveEncoder(i));
      SmartDashboard.putNumber("S"+i+" Drive Velocity", RobotContainer.driveSubsystem.getDriveVelocity(i));
      SmartDashboard.putNumber("S"+i+" Drive EncoderSI", RobotContainer.driveSubsystem.getDriveEncoderSI(i));
      SmartDashboard.putNumber("S"+i+" Drive VelocitySI", RobotContainer.driveSubsystem.getDriveVelocitySI(i));
      SmartDashboard.putNumber("S"+i+" CANCoder Absolute SI", RobotContainer.driveSubsystem.getCancoderAbsoluteSI(i));
      SmartDashboard.putNumber("S"+i+" CANCoder Relative SI", RobotContainer.driveSubsystem.getCancoderRelativeSI(i));
   }
  }

  public void updateChassisTelemetry() {
    SmartDashboard.putString("Chassis Pose", RobotContainer.driveSubsystem.getPose().toString());
    SmartDashboard.putString("Chassis Velocity", RobotContainer.driveSubsystem.getState().speeds.toString());
  }

  public void updateIMUTelemetry() {
    SmartDashboard.putNumber("IMU Yaw", RobotContainer.driveSubsystem.getYaw());
  }

  public void updateAllDisplays(){
    updateIMUTelemetry();
    updateChassisTelemetry();
    updateOdometryTelemetry();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateAllDisplays();
  }
}