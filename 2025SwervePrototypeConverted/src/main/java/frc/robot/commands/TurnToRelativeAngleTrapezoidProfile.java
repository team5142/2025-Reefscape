// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveConstants.SwerveChassis;

public class TurnToRelativeAngleTrapezoidProfile extends Command {

// Software PID turn constants
	//private final double kP = 0.15958;
  private final double kP = 0.11958;
	private final double kI = 0.02;
	private final double kD = 0.0;
	private final double minOmega = 0.37;

  private int counter = 0; 
  private int counterFinal = 200; 
	
  double rotateToAngleRelative;

	DoubleSupplier angleSupplier; // in degrees

	private final double kMaxSpeed = SwerveChassis.MaxAngularRate*180.0/Math.PI; // degrees per second
  private final double kMaxAngularRate = SwerveChassis.MaxAngularRate;
	private final double kMaxAccel = SwerveChassis.maxAngularAcceleration*180.0/Math.PI; // degrees per second square
	//private double kMaxSpeed = 360;
	//private double kMaxAccel = 720;
	private TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(kMaxSpeed, kMaxAccel);
  private final ProfiledPIDController ppc = new ProfiledPIDController(kP, kI, kD, constraints);

  private final double tolerance = 0.5; // degrees of tolerance to end the command
	private final double finalGoal = 0.0;
  private final double minFeedForward = 0.02; // min power to start turning; cannot be less than this


  
  /** Creates a new TurnToRelativeAngleTrapezoidProfile. */
  public TurnToRelativeAngleTrapezoidProfile(double angle, DoubleSupplier angleSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.driveSubsystem);

    rotateToAngleRelative = angle;
    this.angleSupplier = angleSupplier;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
    double initialAngle = angleSupplier.getAsDouble();
    ppc.reset(initialAngle,0); // set current angle and 0 angular velocity as current state
    ppc.setGoal(initialAngle + rotateToAngleRelative);
    System.out.println("initial: " + initialAngle + " " + (initialAngle + rotateToAngleRelative));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    counter++;
    double ff = ppc.calculate(angleSupplier.getAsDouble());
    //System.out.println("FF0:"+ff + " " + kMaxSpeed);
    ff=(ff>=0)?MathUtil.clamp(ff,minFeedForward*kMaxAngularRate, kMaxAngularRate):MathUtil.clamp(ff,-kMaxAngularRate, -minFeedForward*kMaxAngularRate);
    RobotContainer.driveSubsystem.drive(0,0,ff);
    //System.out.println("FF:"+ff);
    
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Turn To Angle Is Done: " + interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ppc.atGoal() || counter > counterFinal;
  }
}
