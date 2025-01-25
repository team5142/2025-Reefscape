// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import frc.robot.Constants.CurrentLimits;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants.Intake;

public class IntakeSubsystem extends SubsystemBase {

  private SparkMax intakeMotor;
  private SparkMaxConfig intakeConfig;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {

    intakeMotor = new SparkMax(11, MotorType.kBrushless);

    intakeConfig = new SparkMaxConfig();

    configureIntakeMotor();

  }

  private void configureIntakeMotor(){
    //In case the spark max needs to be replaced, this method is useful.
    //It configures the motor by adding to the intakeConfig object, 
    //and then applies it, and at the same time resets parameters.
    //NOTE: For PID Testing, we might need to change the persist mode.

  intakeConfig.encoder
    .positionConversionFactor(1)
    .velocityConversionFactor(1);

  intakeConfig.smartCurrentLimit(CurrentLimits.Neo550);
  intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
