// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.CurrentLimiter;
import frc.robot.Constants.EnableCurrentLimiter;
import frc.robot.Constants.EnabledSubsystems;
import frc.robot.Constants.GPMConstants.Arm;

import frc.robot.Constants.GPMConstants.Arm.ArmMotorConstantsEnum;
import frc.robot.Constants.GPMConstants.Arm.ArmPIDConstants;

@SuppressWarnings({ "removal" })

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new GPMSubsystem. */

  public InterpolatingDoubleTreeMap FEED_FORWARD = new InterpolatingDoubleTreeMap();

  // === ARM ====

  // NEO motors connected to Spark Max
  private CANSparkMax armMotorLeft;
  private CANSparkMax armMotorRight;
  private CANSparkMax armMotorLeader;

  private SparkPIDController armPIDControllerLeft;
  private SparkPIDController armPIDControllerRight;
  private SparkPIDController armPidControllerLeader;

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

    armMotorLeft = new CANSparkMax(ArmMotorConstantsEnum.LEFTMOTOR.getArmMotorID(), MotorType.kBrushless);
    armMotorRight = new CANSparkMax(ArmMotorConstantsEnum.RIGHTMOTOR.getArmMotorID(), MotorType.kBrushless);

    // TODO: We will probably have only one Thru-bore encoder, which is sufficient
    // for us; revise the code as needed

    armPIDControllerLeft = armMotorLeft.getPIDController();
    armPIDControllerRight = armMotorRight.getPIDController();

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
  private void configureArmMotors(CANSparkMax motor,  RelativeEncoder encoder, SparkPIDController p, ArmMotorConstantsEnum c,
      CANSparkMax motorToFollow) {

    motor.restoreFactoryDefaults(); //restores the state of the motor to factory defaults
    motor.clearFaults();  //clears a fault that has occurred since the last time the faults were reset
    motor.setInverted(c.getArmMotorInverted()); //sets motor inverted if getArmMotorInverted() returns true

    motor.setIdleMode(IdleMode.kBrake); //sets motor into brake mode
    //motor.setIdleMode(IdleMode.kCoast); 

    encoder.setPositionConversionFactor(Arm.POSITION_CONVERSION_FACTOR);  //sets conversion between NEO units to necessary unit for positon

    motor.setCANTimeout(0); //sets up timeout

    motor.enableVoltageCompensation(Arm.nominalVoltage);  //enables voltage compensation for set voltage [12v]
   
    if (EnableCurrentLimiter.arm) {
      motor.setSmartCurrentLimit(CurrentLimiter.arm); // sets current limit to 40 amps
    }
    
    motor.setOpenLoopRampRate(Arm.rampRate);  // sets the rate to go from 0 to full throttle on open loop
    motor.setClosedLoopRampRate(Arm.rampRate);  // sets the rate to go from 0 to full throttle on open loop

    // sets which motor is the leader and follower; set follower inversion if needed
    if (c.getArmMotorFollower()) {
      motor.follow(motorToFollow,c.getArmMotorInverted());

      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 250);
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 250);

    } else {

      System.out.println("*** Set Arm Leader " + motor.getDeviceId());

      armMotorLeader = motor;
      armEncoderLeader = motor.getEncoder();
      armPidControllerLeader = p;

      //motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 25);
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 50);
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50);


    }

    // --- PID Setup
    // set the PID sensor to motor encoder for hardware PID
    p.setFeedbackDevice(motor.getEncoder());

    // set arm PID coefficients - LIFT
    p.setP(ArmPIDConstants.kP);
    p.setI(ArmPIDConstants.kI);
    p.setD(ArmPIDConstants.kD);
    p.setIZone(ArmPIDConstants.Izone);
    p.setFF(ArmPIDConstants.kF);
    // kMaxOutput = 1 ; range is -1, 1
    p.setOutputRange(-ArmPIDConstants.kMaxOutput, ArmPIDConstants.kMaxOutput);

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
    armMotorLeader.getPIDController().setReference(
        // armEncoderZero is encoder position at ZERO degrees
        // So, the expected encoder position is armEncoderZero plus
        // the degrees angle multiplied by ARM_ENCODER_CHANGE_PER_DEGREE
        (Arm.ARM_ENCODER_CHANGE_PER_DEGREE * angle) + armEncoderZero,
           ControlType.kPosition);

    //System.out.println("AENC-FIN:"+((Arm.ARM_ENCODER_CHANGE_PER_DEGREE * angle) + armEncoderZero));
  }

   public void setArmMotorEncoder(double position) {
    // hold position for encoder value
    armMotorLeader.getPIDController().setReference(
        position,
           ControlType.kPosition);
  } 

  public void stopArmPID() {
    armMotorLeader.getPIDController().setReference((0), ControlType.kVoltage);
  }

  public void armHoldPosition() {
    // TODO: this will depend on the angle; need to calibrate
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
