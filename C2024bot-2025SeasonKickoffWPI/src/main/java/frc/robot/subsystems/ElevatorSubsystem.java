// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SignalsConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.CurrentLimiter;
import frc.robot.Constants.EnableCurrentLimiter;
import frc.robot.Constants.EnabledSubsystems;
import frc.robot.Constants.GPMConstants.Arm;

import frc.robot.Constants.GPMConstants.Arm.ArmMotorConstantsEnum;
import frc.robot.Constants.GPMConstants.Arm.ArmPIDConstants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new GPMSubsystem. */

  public InterpolatingDoubleTreeMap FEED_FORWARD = new InterpolatingDoubleTreeMap();

  // === ARM ====

  // NEO motors connected to Spark Max
  private SparkMax armMotorLeft;
  private SparkMax armMotorRight;
  private SparkMax armMotorLeader;

  private SparkClosedLoopController armPIDControllerLeft;
  private SparkClosedLoopController armPIDControllerRight;
  
  // We wii use built-in NEO encoders for now
  // They're relative, but we can calibrate them based on second Pigeon on the arm
  private RelativeEncoder armEncoderLeft;
  private RelativeEncoder armEncoderRight;
  private RelativeEncoder armEncoderLeader;

  private Pigeon2 armImu;
  private double armEncoderZero;

  // private SparkLimitSwitch noteSensor; //limit switch

  public ElevatorSubsystem() {

    // Check if need to initialize arm
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
