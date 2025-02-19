// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimits;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new GPMSubsystem. */

  private SparkMax leadElevatorMotor;
  private SparkMax followingElevatorMotor;
  private SparkMax secondaryElevatorMotor;

  private SparkMaxConfig leadElevatorConfig;
  private SparkMaxConfig followingElevatorConfig;
  private SparkMaxConfig secondaryElevatorConfig;

  private final double primarykP = 0.01;
  private final double primarykI = 0;
  private final double primarykD = 0.01;

  private final double primaryForwardSpeedLimit = 0.5;
  private final double primaryReverseSpeedLimit = 0.5;

  private final double secondarykP = 0.01;
  private final double secondarykI = 0.01;
  private final double secondarykD = 0.01;

  private final double secondaryForwardSpeedLimit = 0.5;
  private final double secondaryReverseSpeedLimit = 0.5;

  private SparkClosedLoopController leadPID;
  private SparkClosedLoopController secondaryPID;


  /** Creates a new ClimberSubsystem. */
  public ElevatorSubsystem() {

    leadElevatorMotor = new SparkMax(15, MotorType.kBrushless);
    followingElevatorMotor = new SparkMax(16, MotorType.kBrushless);
    //These two control the main stage
    secondaryElevatorMotor = new SparkMax(17, MotorType.kBrushless);
    //This motor controls the second stage


    

    //Initialize PIDS
    leadPID = leadElevatorMotor.getClosedLoopController();
    secondaryPID = secondaryElevatorMotor.getClosedLoopController();


    leadElevatorConfig = new SparkMaxConfig();
    followingElevatorConfig = new SparkMaxConfig();
    secondaryElevatorConfig = new SparkMaxConfig();
    //Only one config is necessary as it will be used for all the canandmags.
   

    configureClimberMotors();
  }

  private void configureClimberMotors(){

   
    
    //Encoders are reset, if we want to go by inches or something we can try multiplying but
    //I think it's easier to just go by rotations and find values with hardware client

    
  
    followingElevatorConfig.follow(15, true); 
      //Follows the lead motor, invert is set to true. We may want to also invert the other motor.

    leadElevatorConfig.smartCurrentLimit(CurrentLimits.Neo500);
    followingElevatorConfig.smartCurrentLimit(CurrentLimits.Neo500);
    secondaryElevatorConfig.smartCurrentLimit(CurrentLimits.Neo500);
      //Applies a 40 amp limit

    leadElevatorConfig.idleMode(IdleMode.kBrake);
    followingElevatorConfig.idleMode(IdleMode.kBrake);
    secondaryElevatorConfig.idleMode(IdleMode.kBrake);
      // Makes it so that the motors stay in their position after being shut off.

    leadElevatorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      // Set PID values for position control. We don't need to pass a closed loop
      // slot, as it will default to slot 0.
      .p(primarykP)
      .i(primarykI)
      .d(primarykD)
      .outputRange(primaryReverseSpeedLimit, primaryForwardSpeedLimit);

    secondaryElevatorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      // Set PID values for position control. We don't need to pass a closed loop
      // slot, as it will default to slot 0.
      .p(secondarykP)
      .i(secondarykI)
      .d(secondarykD)
      .outputRange(secondaryReverseSpeedLimit, secondaryForwardSpeedLimit);


    leadElevatorMotor.configure(leadElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    followingElevatorMotor.configure(followingElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    secondaryElevatorMotor.configure(secondaryElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      //Resets and configures sparkmaxes




   
  }
  
  private void setPrimaryPosition(double position) {

    leadPID.setReference(position, ControlType.kPosition);

  }

  private void setSecondaryPosition(double position) {

    secondaryPID.setReference(position, ControlType.kPosition);
    
  }
  
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //smartdashboard
  }
}
