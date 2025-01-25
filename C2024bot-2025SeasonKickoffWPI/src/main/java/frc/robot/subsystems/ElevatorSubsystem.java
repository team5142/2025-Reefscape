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
import frc.robot.Constants.EnableCurrentLimiter;
import frc.robot.Constants.EnabledSubsystems;
import frc.robot.Constants.GPMConstants.Arm;

import frc.robot.Constants.GPMConstants.Arm.ArmMotorConstantsEnum;
import frc.robot.Constants.GPMConstants.Arm.ArmPIDConstants;
import frc.robot.Constants.CurrentLimits;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new GPMSubsystem. */

  private SparkMax leadElevatorMotor;
  private SparkMaxConfig leadElevatorConfig;
  private SparkMax followingElevatorMotor;
  private SparkMaxConfig followingElevatorConfig;
  private SparkMax secondaryElevatorMotor;
  private SparkMaxConfig secondaryElevatorConfig;
  /** Creates a new ClimberSubsystem. */
  public ElevatorSubsystem() {

    leadElevatorMotor = new SparkMax(10, MotorType.kBrushless);
    followingElevatorMotor = new SparkMax(11, MotorType.kBrushless);
    //These two control the main stage
    secondaryElevatorMotor = new SparkMax(12, MotorType.kBrushless);
    //This motor controls the second stage

    leadElevatorConfig = new SparkMaxConfig();
    followingElevatorConfig = new SparkMaxConfig();
    secondaryElevatorConfig = new SparkMaxConfig();


    configureClimberMotors();
  }

  private void configureClimberMotors(){

    leadElevatorConfig.encoder
    .positionConversionFactor(1)
    .velocityConversionFactor(1);
    followingElevatorConfig.encoder
    .positionConversionFactor(1)
    .velocityConversionFactor(1);
    secondaryElevatorConfig.encoder
    .positionConversionFactor(1)
    .velocityConversionFactor(1);
    //Encoders are reset, if we want to go by inches or something we can try multiplying but
    //I think it's easier to just go by rotations and find values with hardware client

    followingElevatorConfig.follow(10, true); 
    //Follows the lead motor, invert is set to true. We may want to also invert the other motor.

  leadElevatorConfig.smartCurrentLimit(CurrentLimits.Neo500);
  followingElevatorConfig.smartCurrentLimit(CurrentLimits.Neo500);
  secondaryElevatorConfig.smartCurrentLimit(CurrentLimits.Neo500);
    //Applies a 40 amp limit

  leadElevatorMotor.configure(leadElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  followingElevatorMotor.configure(followingElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  secondaryElevatorMotor.configure(secondaryElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //Resets and configures sparkmaxes


  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
