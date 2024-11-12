// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkLowLevel.PeriodicFrame;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimiter;
import frc.robot.Constants.EnableCurrentLimiter;
import frc.robot.Constants.EnabledSubsystems;
import frc.robot.Constants.GPMConstants.Shooter;
import frc.robot.Constants.GPMConstants.Shooter.ShooterMotorConstantsEnum;
import frc.robot.Constants.GPMConstants.Shooter.ShooterPIDConstants;

public class ShooterSubsystem extends SubsystemBase {

  // NEO motors connected to Spark Max
  private SparkMax shooterMotorLeft;
  private SparkMax shooterMotorRight;
  private SparkMax shooterMotorLeader;

  // Built-in NEO encoders
  // Will be used with Velocity PID
  private RelativeEncoder shooterEncoderLeft;
  private RelativeEncoder shooterEncoderRight;

  // Necessary for hardware PID with Spark Max
  private SparkPIDController shooterPIDControllerLeft;
  private SparkPIDController shooterPIDControllerRight;

  private InterpolatingDoubleTreeMap SHOOTER_POWER = new InterpolatingDoubleTreeMap();

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    // Check if need to initialize shooter
    if (! EnabledSubsystems.shooter) { return; }

    shooterMotorLeft = new SparkMax(ShooterMotorConstantsEnum.LEFTMOTOR.getShooterMotorID(), MotorType.kBrushless);
    shooterMotorRight = new SparkMax(ShooterMotorConstantsEnum.RIGHTMOTOR.getShooterMotorID(), MotorType.kBrushless);

    shooterPIDControllerLeft = shooterMotorLeft.getPIDController();
    shooterPIDControllerRight = shooterMotorRight.getPIDController();

    shooterEncoderLeft = shooterMotorLeft.getEncoder();
    shooterEncoderRight = shooterMotorRight.getEncoder();

    // Main Motor; should not follow the other motor
    configureshooterMotors(shooterMotorLeft, shooterEncoderLeft, shooterPIDControllerLeft, ShooterMotorConstantsEnum.LEFTMOTOR, null);
    // Follower Motor
    configureshooterMotors(shooterMotorRight, shooterEncoderRight, shooterPIDControllerRight, ShooterMotorConstantsEnum.RIGHTMOTOR,
        shooterMotorLeft);
    
    //shooterMotorLeft.setIdleMode(IdleMode.kCoast);

    setShooterPower();
    
    System.out.println("*** Shooter initialized");

  }

  /**
   * Configure Shooter motors with a main and a follower
   * 
   * @param motor         - motor object
   * @param p             - PID controller object
   * @param c             - motor constants
   * @param motorToFollow - motor to follow if this is a follower
   */
  private void configureshooterMotors(SparkMax motor, RelativeEncoder encoder, SparkPIDController p, ShooterMotorConstantsEnum c,
      SparkMax motorToFollow) {

    motor.restoreFactoryDefaults();
    motor.clearFaults();
    motor.setInverted(c.getShooterMotorInverted());

    motor.setIdleMode(IdleMode.kBrake);

    encoder.setPositionConversionFactor(Shooter.POSITION_CONVERSION_FACTOR);
    encoder.setVelocityConversionFactor(Shooter.VELOCITY_CONVERSION_FACTOR);

    motor.setCANTimeout(0);

    motor.enableVoltageCompensation(Shooter.nominalVoltage);

    if (EnableCurrentLimiter.shooter) {
      motor.setSmartCurrentLimit(CurrentLimiter.shooter);
    }
    
    motor.setOpenLoopRampRate(Shooter.rampRate);
    motor.setClosedLoopRampRate(Shooter.rampRate);

    // sets which motor is the leader and follower; set follower inversion if needed
    if (c.getShooterMotorFollower()) {
      motor.follow(motorToFollow,c.getShooterMotorInverted());

      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 250);
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 250);

    } else {
      shooterMotorLeader = motor;

      //motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 25);
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 50);
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50);

    }

    // PID Controller setup
    p.setPositionPIDWrappingEnabled(false);
    p.setP(ShooterPIDConstants.kP);
    p.setI(ShooterPIDConstants.kI);
    p.setD(ShooterPIDConstants.kD);
    p.setIZone(ShooterPIDConstants.Izone);
    p.setFF(ShooterPIDConstants.kF);
    // kMaxOutput = 1 ; range is -1, 1
    p.setOutputRange(-ShooterPIDConstants.kMaxOutput, ShooterPIDConstants.kMaxOutput);

    // kMaxOutput = 1 ; range is -1, 1
    // shooterPIDControllerB.setOutputRange(-Constants.GPMConstants.ShooterPIDConstants.kMaxOutput,
    // Constants.GPMConstants.ShooterPIDConstants.kMaxOu
  }

  /**
   * Run shooter with velocity using PID
   * @param speed
   */
  public void runShooterWithVelocity(double speed) {
    shooterMotorLeader.getPIDController().setReference((speed), ControlType.kVelocity);
  }

  // TODO: Modify this
  
  /* 
  public double convertShooterPowerIntoShooterSpeed(double power) {
    return power;
  }
  */

  /* 
  public double convertShooterSpeedIntoShooterPower(double speed) {
    return speed;
  }
  */

  /**
   * Run shooter with NON-PID power -1..1
   * @param power
   */
  public void runShooterWithPower(double power) {
    shooterMotorLeader.set(power);
  }

  /**
   * Run shooter with PID power -1..1; converts power to voltage
   * @param power
   */
  public void runShooterWithPowerPID(double power) {
    runShooterWithVoltagePID(MathUtil.clamp (power * Shooter.nominalVoltage, -Shooter.nominalVoltage, Shooter.nominalVoltage));
  }

  /**
   * Run shooter with PID voltage; clamp voltage to nominal range
   * @param voltage
   */
  public void runShooterWithVoltagePID(double voltage) {
    shooterMotorLeader.getPIDController().setReference(MathUtil.clamp (voltage, -Shooter.nominalVoltage, Shooter.nominalVoltage), ControlType.kVoltage);
  }

  public void stopShooter() {
    //shooterMotorLeader.getPIDController().setReference((0), ControlType.kVelocity);
    shooterMotorLeader.set(0);
  }

  // ===============================
  // ===== Shooter telemetry methods
  // ===============================

  public double getLeftShooterMotorVelocity() {
    return shooterEncoderLeft.getVelocity();
  }

  public double getRightShooterMotorVelocity() {
    return shooterEncoderRight.getVelocity();
  }

    public double getLeftShooterMotorEncoder() {
    return shooterEncoderLeft.getPosition();
  }

  public double getRightShooterMotorEncoder() {
    return shooterEncoderRight.getPosition();
  }

  public void setShooterPower() {
    SHOOTER_POWER.put(0.0, 0.0); //TODO: need to calibrate
    SHOOTER_POWER.put(6.0, 0.8); 
  }

  public double getLeftShooterMotorTemp() {
    return shooterMotorLeft.getMotorTemperature();
  }

  public double getRightShooterMotorTemp() {
    return shooterMotorRight.getMotorTemperature();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
