// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.SwerveConstants.SwerveChassis;
import frc.robot.Constants.SwerveConstants.SysIdConstants;
import frc.robot.Constants.SwerveConstants.TunerConstants;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveConstants.SwerveChassis.SwerveModuleConstantsEnum;
import frc.robot.Constants.SwerveConstants.SwerveChassis;

public class DriveSubsystem extends SwerveDrivetrain implements Subsystem {

  private SwerveModuleConstants[] swerveModuleConstants;

  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean hasAppliedOperatorPerspective = false;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(SwerveChassis.MaxSpeed * 0.1).withRotationalDeadband(SwerveChassis.MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop

  Pigeon2 imu;
  private double trajectoryAdjustmentIMU; // This is the value we need to adjust the IMU by after Trajectory
  // is completed

  private double previousOmegaRotationCommand;

  

  private final SysIdRoutine mSysIdRoutine = 
    new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(Volts.of(0.1).per(Seconds.of(1)),
           Volts.of(0.1),
           Seconds.of(50)),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motor(s).
              (Measure<Voltage> volts) -> {
                        this.turnToAngleWithVolt(volts.in(Volts));
                },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the shooter motor.
                log.motor("chassis-angleMotor")
                    .voltage(Volts.of(previousOmegaRotationCommand))
                    .angularPosition(Radians.of(this.getAngularPosition()))
                    .angularVelocity(RadiansPerSecond.of(this.getAngularVelocity()));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("shooter")
              this));

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(double OdometryUpdateFrequency) {
        super(TunerConstants.DrivetrainConstants, OdometryUpdateFrequency, configureSwerveChassis());
        imu = this.getPigeon2();
    }

  public DriveSubsystem() {
        super(TunerConstants.DrivetrainConstants, configureSwerveChassis());
        imu = this.getPigeon2();

        this.registerTelemetry(this::telemeterize);
    }

   /**
   * Returns a command that will execute a quasistatic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return mSysIdRoutine.quasistatic(direction);
  }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return mSysIdRoutine.dynamic(direction);
  }

  public static SwerveModuleConstants[] configureSwerveChassis() {
    return new SwerveModuleConstants[] {
        TunerConstants.ConstantCreator.createModuleConstants(
            SwerveModuleConstantsEnum.MOD0.getAngleMotorID(),
            SwerveModuleConstantsEnum.MOD0.getDriveMotorID(),
            SwerveModuleConstantsEnum.MOD0.getCancoderID(),
            SwerveModuleConstantsEnum.MOD0.getAngleOffset(),
            SwerveChassis.WHEEL_BASE / 2.0,
            SwerveChassis.TRACK_WIDTH / 2.0,
            SwerveModuleConstantsEnum.MOD0.isDriveMotorInverted())
            .withSteerMotorInverted(SwerveModuleConstantsEnum.MOD0.isAngleMotorInverted())
            ,

        TunerConstants.ConstantCreator.createModuleConstants(
            SwerveModuleConstantsEnum.MOD1.getAngleMotorID(),
            SwerveModuleConstantsEnum.MOD1.getDriveMotorID(),
            SwerveModuleConstantsEnum.MOD1.getCancoderID(),
            SwerveModuleConstantsEnum.MOD1.getAngleOffset(),
            SwerveChassis.WHEEL_BASE / 2.0,
            -SwerveChassis.TRACK_WIDTH / 2.0,
            SwerveModuleConstantsEnum.MOD1.isDriveMotorInverted())
            .withSteerMotorInverted(SwerveModuleConstantsEnum.MOD1.isAngleMotorInverted())
            ,

        TunerConstants.ConstantCreator.createModuleConstants(
            SwerveModuleConstantsEnum.MOD2.getAngleMotorID(),
            SwerveModuleConstantsEnum.MOD2.getDriveMotorID(),
            SwerveModuleConstantsEnum.MOD2.getCancoderID(),
            SwerveModuleConstantsEnum.MOD2.getAngleOffset(),
            -SwerveChassis.WHEEL_BASE / 2.0,
            SwerveChassis.TRACK_WIDTH / 2.0,
            SwerveModuleConstantsEnum.MOD2.isDriveMotorInverted())
            .withSteerMotorInverted(SwerveModuleConstantsEnum.MOD2.isAngleMotorInverted())
            ,

        TunerConstants.ConstantCreator.createModuleConstants(
            SwerveModuleConstantsEnum.MOD3.getAngleMotorID(),
            SwerveModuleConstantsEnum.MOD3.getDriveMotorID(),
            SwerveModuleConstantsEnum.MOD3.getCancoderID(),
            SwerveModuleConstantsEnum.MOD3.getAngleOffset(),
            -SwerveChassis.WHEEL_BASE / 2.0,
            -SwerveChassis.TRACK_WIDTH / 2.0,
            SwerveModuleConstantsEnum.MOD3.isDriveMotorInverted())
            .withSteerMotorInverted(SwerveModuleConstantsEnum.MOD3.isAngleMotorInverted())
    };
  }

  public void drive(double xVelocity_m_per_s, double yVelocity_m_per_s, double omega_rad_per_s) {
    //System.out.println("o:" + omega_rad_per_s/SwerveChassis.MaxAngularRate);
    this.setControl(
      drive.withVelocityX(xVelocity_m_per_s)
        .withVelocityY(yVelocity_m_per_s)
        .withRotationalRate(omega_rad_per_s)
    );
    previousOmegaRotationCommand = omega_rad_per_s / SwerveChassis.MaxAngularRate;
  }

    /** 
   * Field Centric Pose of the chassis
   * We get it from odometry, rather than sensors. That means commands that use it must ensure that
   * odometry was properly updated.
  */
  public Pose2d getPose() {
    return this.getState().Pose;
  }

  public ChassisSpeeds getChassisSpeeds() {
    return this.getState().speeds;
  }

  public void driveWithChassisSpeeds(ChassisSpeeds speeds) {
    drive(
      speeds.vxMetersPerSecond,
      speeds.vyMetersPerSecond,
      speeds.omegaRadiansPerSecond
    );
  }

  public TalonFX getDriveMotor(int modNum){
    return this.getModule(modNum).getDriveMotor();
  }

  public TalonFX getTurnMotor(int modNum){
    return this.getModule(modNum).getSteerMotor();
  }

  public double getDriveEncoder(int modNum){
    return this.getModule(modNum).getDriveMotor().getRotorPosition().getValueAsDouble();
  }

  public double getDriveEncoderSI(int modNum){
    return this.getModule(modNum).getDriveMotor().getRotorPosition().getValueAsDouble()
      *SwerveChassis.metersPerRotationFX;
  }

  public double getTurnEncoder(int modNum){
    return this.getModule(modNum).getSteerMotor().getRotorPosition().getValueAsDouble();
  }

  public double getDriveVelocity(int modNum){
    return this.getModule(modNum).getDriveMotor().getRotorVelocity().getValueAsDouble();
  }

 public double getDriveVelocitySI(int modNum){
    return this.getModule(modNum).getDriveMotor().getRotorVelocity().getValueAsDouble()
      *SwerveChassis.metersPerRotationFX;
  } 

  public double getTurnVelocity(int modNum){
    return this.getModule(modNum).getSteerMotor().getRotorVelocity().getValueAsDouble();
  }

  public double getCancoderAbsolute(int modNum){
    return this.getModule(modNum).getCANcoder().getAbsolutePosition().getValueAsDouble();
  }

  public double getCancoderRelative(int modNum){
    return this.getModule(modNum).getCANcoder().getPosition().getValueAsDouble();
  }

   public double getCancoderAbsoluteSI(int modNum){
    return this.getModule(modNum).getCANcoder().getAbsolutePosition().getValueAsDouble()*360.0;
  }

  public double getCancoderRelativeSI(int modNum){
    return (this.getModule(modNum).getCANcoder().getPosition().getValueAsDouble()
            - getModuleEnum(modNum).getAngleOffset())*360.0;
  }

  public SwerveModuleConstantsEnum getModuleEnum(int modNum){
    switch(modNum){
      case 0: 
        return SwerveModuleConstantsEnum.MOD0;
      case 1: 
        return SwerveModuleConstantsEnum.MOD1;
      case 2:
        return SwerveModuleConstantsEnum.MOD2;
      default:
        return SwerveModuleConstantsEnum.MOD3;
    }
  }

  public void stopRobot(){
    drive(0,0,0);
  }

  public double getOmegaRoationCommand() {
    return previousOmegaRotationCommand;
  }
  /**
   * Note that all IMU methods that take or return values should do so in SI
   * units.
   */

  public double getPitch() {
    // double[] ypr = new double[3];
    // pigeon2.getYawPitchRoll(ypr);
    // return ypr[1];

    // Front UP - positive Pitch
    return -imu.getPitch().getValueAsDouble();
  }

  /**
   * Gets the roll of the robot (Y axis rotation) (roll is the leaning around the
   * axis that goes straight forward)
   * 
   * @return
   */
  public double getRoll() {
    // double[] ypr = new double[3];
    // pigeon2.getYawPitchRoll(ypr);
    // return ypr[2];

    // Left UP - positive Roll
    return imu.getRoll().getValueAsDouble();
  }

  /**
   * Gets the yaw of the robot (Z axis rotation) (yaw is the direction that the
   * robot is facing around an axis that shoots straight up)
   * 
   * @return
   */
  public double getYaw() {
    // double[] ypr = new double[3];
    // pigeon2.getYawPitchRoll(ypr);
    // System.out.println(ypr[0]);
    // return ypr[0];

    return imu.getYaw().getValueAsDouble(); // With Pigeon2 this method returns values in degrees
  }

  public Rotation2d getYawRotation2d() {
    return Rotation2d.fromDegrees(getYaw());
  }

  /**
   * Zeroes the yaw of the robot
   * 
   * @return The previous yaw
   */
  public double zeroYaw() {
    double previousYaw = getYaw();
    if (RobotContainer.isAllianceRed && RobotContainer.isReversingControllerAndIMUForRed) {
      imu.setYaw(180.0);
    } else {
      imu.setYaw(0);
    }
    return previousYaw;
  }

  public double setYaw(double y) {
    double previousYaw = getYaw();
    imu.setYaw(y);
    return previousYaw;
  }

  public Rotation2d getRotation2d() {
    return imu.getRotation2d();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    if (RobotContainer.isAllianceRed && RobotContainer.isReversingControllerAndIMUForRed) {
      imu.setYaw(180.0);
    } else {
      imu.setYaw(0);
    }
    System.out.println("Yaw and Fused Heading set");
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180 in Rotation2d format
   */
  public Rotation2d getHeading() {
    return imu.getRotation2d();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -imu.getRate();
  }

  /**
   * This method is used when we want to "snap" the chassis to a trajectory start,
   * meaning
   * assuming that the robot is at the starting point of the trajectory.
   * Here we remember starting Yaw before trajectory so it can be restored
   * back after trajectory
   * 
   * @param y - starting Yaw of the trajectory
   * @return - old value of the Yaw (we do not currently use it)
   */
  public double setYawForTrajectory(double y) {
    trajectoryAdjustmentIMU = this.getPose().getRotation().getDegrees() - y;

    // alex test
    System.out.println("---- Trajectory AdjustmentIMU: "+ trajectoryAdjustmentIMU
      + " CP: "+ this.getPose().getRotation().getDegrees() + " A: " + y) ;

    return y; // our own setYaw that returns old angle
  }

  /**
   * Once the trajectory is done, we want to readjust the Yaw considering the
   * value that we "remember", so
   * the field-centric drive axis will not change. That may allow one to drive
   * automated trajectories in teleop
   * without losing the Yaw direction.
   */
  public void restoreYawAfterTrajectory() {
    System.out.println("Final pose: " + this.getPose());
    System.out.println(
        "Restoring original IMU after trajectory " + (this.getPose().getRotation().getDegrees() + trajectoryAdjustmentIMU));
    this.seedFieldRelative(
      new Pose2d(0, 0, 
        new Rotation2d().fromDegrees(this.getPose().getRotation().getDegrees() + trajectoryAdjustmentIMU)));
  }

  /**
   * Set odometry to a specified field-centric Pose2d
   * You may need to do so for the trajectory driving, if you want the robot to assume being at the
   * start of the trajectory.
   * Be aware that on-going odometry updates use IMU. So, your odometry yaw may change incorrectly
   * later if the current yaw is not reset properly on the IMU first.
   */
  public void resetOdometry(Pose2d pose) {
    this.seedFieldRelative(pose);
  }

  /* Accept the swerve drive state and telemeterize it to smartdashboard */
  public void telemeterize(SwerveDriveState state) {
    SmartDashboard.putString("P:", state.Pose.toString());
    // SmartDashboard.putString("M0A:", state.ModuleStates[0].toString());
    // SmartDashboard.putString("M1A:", state.ModuleStates[1].toString());

  }

  public void turnToAngleWithVolt(double power) {
    drive(0, 0, power*SwerveChassis.MaxAngularRate);
  }

  public double getAngularPosition() {
    return this.getState().Pose.getRotation().getRadians();
  }

  public double getAngularVelocity() {
    return this.getState().speeds.omegaRadiansPerSecond;
  }

  @Override
  public void periodic() {
    /* Periodically try to apply the operator perspective */
        /* If we haven't applied the operator perspective before, then we should apply it regardless of DS state */
        /* This allows us to correct the perspective in case the robot code restarts mid-match */
        /* Otherwise, only check and apply the operator perspective if the DS is disabled */
        /* This ensures driving behavior doesn't change until an explicit disable event occurs during testing*/
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                this.setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red ? SwerveChassis.redAlliancePerspectiveRotation
                                : SwerveChassis.blueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
            });
        }
  }
}
