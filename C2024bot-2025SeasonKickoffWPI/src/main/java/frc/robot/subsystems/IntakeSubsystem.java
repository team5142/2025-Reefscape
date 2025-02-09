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
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants.Intake;

public class IntakeSubsystem extends SubsystemBase {

  private Canandcolor leftCoralSensor;
  private Canandcolor rightCoralSensor;
  private Canandcolor algaeSensor;

  private final double algaeProximityThreshold = 0.5;
  private final double coralProximityThreshold = 0.5;

  private SparkMax coralMotor;
  private SparkMaxConfig coralConfig;
  
  private SparkMax algaeMotor;
  private SparkMaxConfig algaeConfig;

  private final double algaekP = 0.01;
  private final double algaekI = 0.01;
  private final double algaekD = 0.01;

  private final double algaeForwardSpeedLimit = 0.5;
  private final double algaeReverseSpeedLimit = 0.5;

  private final double algaeIntakeSpeed = 1;
  private final double algaeEjectSpeed = 1;

  private final double coralIntakeSpeed = 1;
  private final double coralEjectSpeed = 1;

  private RelativeEncoder algaeEncoder;

  private SparkClosedLoopController algaePID;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {

    coralMotor = new SparkMax(14, MotorType.kBrushless);
    algaeMotor = new SparkMax(15, MotorType.kBrushless);

    coralConfig = new SparkMaxConfig();
    algaeConfig = new SparkMaxConfig();

    algaeEncoder = algaeMotor.getEncoder();

    algaePID = algaeMotor.getClosedLoopController();

    configureIntakeMotors();

    rightCoralSensor = new Canandcolor(17);
    leftCoralSensor = new Canandcolor(18);
    algaeSensor = new Canandcolor(19);

  }

  private void configureIntakeMotors(){
    //In case the spark max needs to be replaced, this method is useful.
    //It configures the motor by adding to the intakeConfig object, 
    //and then applies it, and at the same time resets parameters.
    //NOTE: For PID Testing, we might need to change the persist mode.

    coralConfig.encoder
    .positionConversionFactor(1)
    .velocityConversionFactor(1);
    algaeConfig.encoder
    .positionConversionFactor(1)
    .velocityConversionFactor(1);

  coralConfig.smartCurrentLimit(CurrentLimits.Neo550);
  algaeConfig.smartCurrentLimit(CurrentLimits.Neo550);
  
   algaeConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      // Set PID values for position control. We don't need to pass a closed loop
      // slot, as it will default to slot 0.
      .p(algaekP)
      .i(algaekI)
      .d(algaekD)
      .outputRange(algaeReverseSpeedLimit, algaeForwardSpeedLimit);

  coralMotor.configure(coralConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  algaeMotor.configure(algaeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  //methods to be called in commands and as instant commands
  
  private void holdAlgae(){

    algaePID.setReference(algaeEncoder.getPosition(), ControlType.kPosition);
    //sets the reference to the current position, so that it keeps the motor in the same place once the algae is intaked and keeps it secured.
  }

  private void intakeAlgae(){

    algaeMotor.set(algaeIntakeSpeed);

  }

  private void ejectAlgae(){

    algaeMotor.set(algaeEjectSpeed);

  }

  private void intakeCoral(){

    coralMotor.set(coralIntakeSpeed);

  }

  private void ejectCoral(){

    coralMotor.set(coralEjectSpeed);

  }

  private boolean isAlgaeIntaked(){

    return algaeSensor.getProximity() < algaeProximityThreshold; 
    //algae sensor returns a value between 0 and 1, and we will find the value that determines if the algae is present.

  }

  private boolean isLeftCoralIntaked(){

    return leftCoralSensor.getProximity() < coralProximityThreshold;

  }

  private boolean isRightCoralIntaked(){

    return rightCoralSensor.getProximity() < coralProximityThreshold;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // display isAlgaeIntaked() and left and right coral to the SmartDashboard
  }
}
