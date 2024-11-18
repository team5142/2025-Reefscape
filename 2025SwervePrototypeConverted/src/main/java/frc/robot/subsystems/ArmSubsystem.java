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

public class ArmSubsystem extends SubsystemBase {
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

  public ArmSubsystem() {

    // Check if need to initialize arm
    if (!EnabledSubsystems.arm) {
      return;
    }

    System.out.println("*** Initializing Arm Subsystem...");

    // ==========================
    // === ARM initialization
    // ==========================

    armMotorLeft = new SparkMax(ArmMotorConstantsEnum.LEFTMOTOR.getArmMotorID(), MotorType.kBrushless);
    armMotorRight = new SparkMax(ArmMotorConstantsEnum.RIGHTMOTOR.getArmMotorID(), MotorType.kBrushless);


    armPIDControllerLeft = armMotorLeft.getClosedLoopController();
    armPIDControllerRight = armMotorRight.getClosedLoopController();

    // Set Arm encoders
    armEncoderLeft = armMotorLeft.getEncoder();
    armEncoderRight = armMotorRight.getEncoder();

    // Main Motor; should not follow the other motor
    configureArmMotors(armMotorLeft, armEncoderLeft, armPIDControllerLeft, ArmMotorConstantsEnum.LEFTMOTOR, null);
    // Follower Motor
    configureArmMotors(armMotorRight, armEncoderRight, armPIDControllerRight, ArmMotorConstantsEnum.RIGHTMOTOR, armMotorLeft);

    //armMotorRight.setIdleMode(IdleMode.kCoast);

    // ==========================
    // === ARM IMU initialization
    // ==========================
    // This IMU should be attached FLAT to the ARM, with X pointing straight
    // forward.
    // The IMU angle will allow us to calibrate NEO encoders rotating the arm.
    armImu = new Pigeon2(Arm.PIGEON2_ARM_CAN_ID, "rio");

    calibrateArmEncoderToPitch();

    setArmFeedForward();

    System.out.println("*** Arm Subsystem Initialized");

  }

  /**
   * Configure Arm motors with a main and a follower
   * 
   * @param motor         - motor object
   * @param p             - PID controller object
   * @param c             - motor constants
   * @param motorToFollow - motor to follow if this is a follower
   */
  private void configureArmMotors(SparkMax motor,  RelativeEncoder encoder, SparkClosedLoopController p, ArmMotorConstantsEnum c,
      SparkMax motorToFollow) {

    SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();

    //motor.restoreFactoryDefaults(); //restores the state of the motor to factory defaults
    motor.clearFaults();  //clears a fault that has occurred since the last time the faults were reset
    motor.setInverted(c.getArmMotorInverted()); //sets motor inverted if getArmMotorInverted() returns true

    sparkMaxConfig.idleMode(IdleMode.kBrake); //sets motor into brake mode
    //motor.setIdleMode(IdleMode.kCoast); 

    EncoderConfig encoderConfig = new EncoderConfig();
    encoderConfig.positionConversionFactor(Arm.POSITION_CONVERSION_FACTOR);  //sets conversion between NEO units to necessary unit for positon
    sparkMaxConfig.apply(encoderConfig);

    motor.setCANTimeout(0); //sets up timeout

    sparkMaxConfig.voltageCompensation(Arm.nominalVoltage);  //enables voltage compensation for set voltage [12v]
   
    if (EnableCurrentLimiter.arm) {
      sparkMaxConfig.smartCurrentLimit(CurrentLimiter.arm); // sets current limit to 40 amps
    }
    
    sparkMaxConfig.openLoopRampRate(Arm.rampRate);  // sets the rate to go from 0 to full throttle on open loop
    sparkMaxConfig.closedLoopRampRate(Arm.rampRate);  // sets the rate to go from 0 to full throttle on open loop


    SignalsConfig signalsConfig = new SignalsConfig();

    // sets which motor is the leader and follower; set follower inversion if needed
    if (c.getArmMotorFollower()) {
      sparkMaxConfig.follow(motorToFollow,c.getArmMotorInverted());

      //motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
      //motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 250);
      //motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 250);

      // kstatus0
      signalsConfig.faultsPeriodMs(100);
      signalsConfig.appliedOutputPeriodMs(100);
      signalsConfig.outputCurrentPeriodMs(100);

      // kstatus1
      signalsConfig.motorTemperaturePeriodMs(250);
      signalsConfig.primaryEncoderVelocityPeriodMs(250);

      //kstatus2
      signalsConfig.primaryEncoderPositionPeriodMs(250);


      // status frame meaning:
      //m_drivingSparkMax.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus0, 55);      // Applied *** Output, Faults, Stick Faults, Is Follower
      //m_drivingSparkMax.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus1, 15);      // Motor Velocity, Motor Tempature, Motor Voltage, Motor Current
      //m_drivingSparkMax.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus2, 15);      // Motor Position
      //m_drivingSparkMax.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus3, 65535);   // Analog Sensor Voltage, Velocity, Position
      //m_drivingSparkMax.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus4, 65535);   // Alternate Encoder Velocity, Position
      //m_drivingSparkMax.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus5, 65535);   // Duty Cycle Absolute Encoder Position, Absolute Angle
      //m_drivingSparkMax.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus6, 65565);   // Duty Cycle Absolute Encoder Velocity, Frequency

    } else {

      System.out.println("*** Set Arm Leader " + motor.getDeviceId());

      armMotorLeader = motor;
      armEncoderLeader = motor.getEncoder();

      //motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 25);
      //motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 50);
      //motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50);

      // kstatus1
      signalsConfig.motorTemperaturePeriodMs(50);
      signalsConfig.primaryEncoderVelocityPeriodMs(50);

      // kstatus2
      signalsConfig.primaryEncoderPositionPeriodMs(50);


    }

    // apply signals
    sparkMaxConfig.apply(signalsConfig);

    ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();
    // --- PID Setup
    // set the PID sensor to motor encoder for hardware PID
    closedLoopConfig.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    // set arm PID coefficients - LIFT
    closedLoopConfig.p(ArmPIDConstants.kP);
    closedLoopConfig.i(ArmPIDConstants.kI);
    closedLoopConfig.d(ArmPIDConstants.kD);
    closedLoopConfig.iZone(ArmPIDConstants.Izone);
    //p.setFF(ArmPIDConstants.kF);
    // kMaxOutput = 1 ; range is -1, 1
    closedLoopConfig.outputRange(-ArmPIDConstants.kMaxOutput, ArmPIDConstants.kMaxOutput);

    // Apply closed loop configuration
    sparkMaxConfig.apply(closedLoopConfig);

    motor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  // ======== ARM METHODS

  /**
   * The NEO built-in encoder cannot be reset.
   * So, instead we remember its "Zero position" using the IMU
   * 
   * 1. get the pitch from arm IMU
   * 2. subtract pan IMU from Arm IMU if desired (to get an angle relative to pan rather than the floor in case of uneven surface)
   * 3. multiply the adjusted pitch by the ARM_ENCODER_CHANGE_PER_DEGREE to translate the angle into encoder units
   * 4. subtract encoder units obtained in #3 from the current encoder setting. That gives us encoder setting at 0 angle
   */
  private void calibrateArmEncoderToPitch() {
    // The 0-degree encoder position is current encoder value minus (degrees times encoderchange_per_degree)
    // We also may decide to apply a correction for an uneven floor by subtracting pan IMU pitch from
    // arm IMU pitch, which gives us relative angle of the arm to the pan
    armEncoderZero = getArmEncoderLeader() -
      (
       (getArmIMUPitch() - 
          ((Arm.USE_PAN_IMU_FOR_CORRECTION) ? RobotContainer.driveSubsystem.getPitch() : 0))  // pan IMU Pitch-based correction for uneven surface
            * Arm.ARM_ENCODER_CHANGE_PER_DEGREE
      );

    // Alex test
    //System.out.println("ARM0: "+ armEncoderZero);
  }

   public void resetArmEncoderToPitch() {
    // The 0-degree encoder position is current encoder value minus (degrees times encoderchange_per_degree)
    // We also may decide to apply a correction for an uneven floor by subtracting pan IMU pitch from
    // arm IMU pitch, which gives us relative angle of the arm to the pan
    armEncoderZero = getArmEncoderLeader() -
      (
       (Arm.ARM_IMU_RESET_ANGLE - 
          ((Arm.USE_PAN_IMU_FOR_CORRECTION) ? RobotContainer.driveSubsystem.getPitch() : 0))  // pan IMU Pitch-based correction for uneven surface
            * Arm.ARM_ENCODER_CHANGE_PER_DEGREE
      );

    // Alex test
    //System.out.println("ARM0: "+ armEncoderZero);
  }

  /**
   * Get Arm IMU pitch in degrees.
   * IMU is pointed to the FRONT of the robot with the X and left with Y
   * Positive Pitch angle increases when arm is going from front towards the back.
   * 
   * @return
   */
  public double getArmIMUPitch() {
    return -armImu.getPitch().getValueAsDouble();
  }

  /**
   * Get current angle of the arm.
   * Positive Pitch angle increases when arm is going from front towards the back.
   * The pitch (per encoders) is a difference between current encoder value and a
   * "zero degree" encoder value
   * divided by ARM_ENCODER_CHANGE_PER_DEGREE
   * 
   * @return - degrees angle
   */
  public double getArmAngleSI() {
    return (armEncoderLeader.getPosition() - armEncoderZero) / Arm.ARM_ENCODER_CHANGE_PER_DEGREE;
  }

  // Encoder telemetry
  public double getArmEncoderLeft() {
    return armEncoderLeft.getPosition();
  }

  public double getArmEncoderRight() {
    return armEncoderRight.getPosition();
  }

  public double getArmEncoderLeader() {
    return armEncoderLeader.getPosition();
  }

  public double getFormFeedPowerForCurrentAngle() {
    return FEED_FORWARD.get(getArmAngleSI());
  }

  /**
   * Set Arm motor to degrees Angle using PID
   * Positive Pitch angle increases is when arm is going from front towards the
   * back.
   * 
   * @param angle
   */
  public void setArmMotorAnglesSI(double angle) {
    //armPidControllerLeader.setFF(FEED_FORWARD.get(angle));
    armMotorLeader.getClosedLoopController().setReference(
        // armEncoderZero is encoder position at ZERO degrees
        // So, the expected encoder position is armEncoderZero plus
        // the degrees angle multiplied by ARM_ENCODER_CHANGE_PER_DEGREE
        (Arm.ARM_ENCODER_CHANGE_PER_DEGREE * angle) + armEncoderZero,
           ControlType.kPosition);

    //System.out.println("AENC-FIN:"+((Arm.ARM_ENCODER_CHANGE_PER_DEGREE * angle) + armEncoderZero));
  }

   public void setArmMotorEncoder(double position) {
    // hold position for encoder value
    armMotorLeader.getClosedLoopController().setReference(
        position,
           ControlType.kPosition);
  } 

  public void stopArmPID() {
    armMotorLeader.getClosedLoopController().setReference((0), ControlType.kVoltage);
  }

  public void armHoldPosition() {
    armMotorLeader.set(0);
  }

  // ==================================
  // test methods; for calibration only
  // ==================================
  public void runArmMotors(double power) {
    armMotorLeader.set(power);
    //armMotorLeft.set(power);
    //armMotorRight.set(power);
  }

  public void stopArmMotors() {
    //armMotorLeader.set(0);
    armMotorLeft.set(0);
    armMotorRight.set(0);
  }

  public double getLeftArmMotorEncoder() {
    return armEncoderLeft.getPosition();
  }

  public double getRightArmMotorEncoder() {
    return armEncoderRight.getPosition();
  }

  public double getLeftArmMotorVelocity() {
    return armEncoderLeft.getVelocity();
  }

  public double getRightArmMotorVelocity() {
    return armEncoderRight.getVelocity();
  }

  public double getLeaderArmMotorEncoder() {
    return armEncoderLeader.getPosition();
  }

  public void setArmFeedForward() {
    FEED_FORWARD.put(-90.0,0.015);
    FEED_FORWARD.put(-82.7,0.015);
    FEED_FORWARD.put(-25.0,0.015);
    FEED_FORWARD.put(-20.0,0.015);
    FEED_FORWARD.put(-10.0,0.007);
    FEED_FORWARD.put(-5.0,0.007);
    FEED_FORWARD.put(0.0,0.0);
    FEED_FORWARD.put(5.0, -0.007);
    FEED_FORWARD.put(15.0, -0.007);
    FEED_FORWARD.put(20.0, -0.009);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
