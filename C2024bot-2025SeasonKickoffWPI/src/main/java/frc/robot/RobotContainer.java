// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.GPMConstants.Arm;
import frc.robot.Constants.OIConstants.ControllerDevice;
import frc.robot.Constants.SwerveConstants.Intake;
import frc.robot.Constants.VisionConstants.PhotonVisionConstants;

import frc.robot.commands.AutonomousTrajectory2Poses;
import frc.robot.commands.DriveManuallyCommand;

import frc.robot.commands.RunTrajectorySequenceRobotAtStartPoint;

import frc.robot.commands.StopRobot;
import frc.robot.commands.TurnToRelativeAngleSoftwarePIDCommand;
import frc.robot.commands.TurnToRelativeAngleTrapezoidProfile;
import frc.robot.lib.GPMHelpers;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LLVisionSubsystem;
import frc.robot.subsystems.SmartDashboardSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static Controller xboxDriveController;
  public static Controller xboxGPMController;
  public static final DriveSubsystem driveSubsystem = new DriveSubsystem();
  public static final ElevatorSubsystem armSubsystem = new ElevatorSubsystem();
  public static final SmartDashboardSubsystem smartDashboardSubsystem = new SmartDashboardSubsystem();
  public static final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public final static GPMHelpers gpmHelpers = new GPMHelpers();
  public static final LLVisionSubsystem llVisionSubsystem = new LLVisionSubsystem();

  public static boolean isAllianceRed = false;
  public static boolean isReversingControllerAndIMUForRed = true;

  public static Controller driveStick; // for robot testing only
  public static Controller driveStick1; // for robot testing only
  public static Controller driveStick2; // for robot testing only
  public static Controller driveStick3; // for robot testing only

  // Replace with CommandPS4Controller or CommandJoystick if needed
  // private final CommandXboxController m_driverController =
  //     new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(){ 
    // Configure driver interface - binding joystick objects to port numbers
    configureDriverInterface();

      // Configure the trigger bindings
    configureBindings();

     driveSubsystem.setDefaultCommand(
        new DriveManuallyCommand(
            () -> getDriverXAxis(),
            () -> getDriverYAxis(),
            () -> getDriverOmegaAxis()));
  }



  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings()  {
    // testAutoOdometry();
    // testCharacterization();
    //testTurn();
    //allTestCommandsGPM();
    // testAutoOdometry();
    //allTestCommandsDrive();
    //testNotePickup();
    // try {
    //   testAuto();
    // }
    // catch (Exception e) {
    //   System.out.println("test auto error: " + e);
    // }

  }

  private void configureDriverInterface(){
    xboxDriveController = new Controller(ControllerDevice.XBOX_CONTROLLER);
    xboxGPMController = new Controller(ControllerDevice.XBOX_CONTROLLER_GPM);
  }
  








  private void testAutoOdometry() {
    new JoystickButton(xboxDriveController, 1)
      .onTrue(new InstantCommand(() -> RobotContainer.driveSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))));
  }

  private void testTurn() {
    new JoystickButton(xboxDriveController, 1)
      .onTrue(new TurnToRelativeAngleSoftwarePIDCommand(()->new Rotation2d().fromDegrees(1.5)))
      .onFalse(new StopRobot());
  }

  public void testAuto() throws Exception {
    new JoystickButton(xboxDriveController, 2)
      .onTrue(new RunTrajectorySequenceRobotAtStartPoint("OneMeterForward"))
      .onFalse(new StopRobot());
    new JoystickButton(xboxDriveController, 3)
      .onTrue(new RunTrajectorySequenceRobotAtStartPoint("OneMeterRight"))
      .onFalse(new StopRobot());
    new JoystickButton(xboxDriveController, 4)
      .onTrue(new RunTrajectorySequenceRobotAtStartPoint("Turn90"))
      .onFalse(new StopRobot());
    // new JoystickButton(xboxDriveController, 6)
    //   .onTrue(new RunTrajectorySequenceRobotAtStartPoint("FourMeterForward"))
    //   .onFalse(new StopRobot());

    // new JoystickButton(xboxDriveController, 6)
    //   .onTrue(new RunTrajectorySequenceRobotAtStartPoint("OneMeterDiagonal"))
    //   .onFalse(new StopRobot());

    new JoystickButton(xboxDriveController, 6)
      .onTrue(new RunTrajectorySequenceRobotAtStartPoint("2MeterForward90Degrees"))
      .onFalse(new StopRobot());


    new JoystickButton(xboxDriveController, 5)
      .onTrue(new AutonomousTrajectory2Poses(new Pose2d(1, 1, new Rotation2d(0)), 
        new Pose2d(2, 2, new Rotation2d(0))))
      .onFalse(new StopRobot());
  }

  public void testCharacterization() {
    new JoystickButton(xboxDriveController, 2)
      .onTrue(driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward))
      .onFalse(new StopRobot());
    new JoystickButton(xboxDriveController, 3)
      .onTrue(driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse))
      .onFalse(new StopRobot());
    new JoystickButton(xboxDriveController, 4)
      .onTrue(driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward))
      .onFalse(new StopRobot());
    new JoystickButton(xboxDriveController, 5)
      .onTrue(driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse))
      .onFalse(new StopRobot());
  }

    // Alliance color determination
  public void checkAllianceColor() {
    SmartDashboard.putString("AllianceColor", DriverStation.getAlliance().toString());
  }

  public static void setIfAllianceRed() {
    var alliance = DriverStation.getAlliance();
    if (! alliance.isPresent()) {
        System.out.println("=== !!! Alliance not present !!! === Staying with the BLUE system");
    } else {
        isAllianceRed = alliance.get() == DriverStation.Alliance.Red;
        System.out.println("*** RED Alliance: "+isAllianceRed);
    }
  }
  public static void toggleReversingControllerAndIMUForRed() {
    isReversingControllerAndIMUForRed = !isReversingControllerAndIMUForRed;
  }

  // Driver preferred controls
  private double getDriverXAxis() {
    //return -xboxController.getLeftStickY();
    return -xboxDriveController.getRightStickY();
  }

  private double getDriverYAxis() {
    //return -xboxController.getLeftStickX();
    return -xboxDriveController.getRightStickX();
  }

  private double getDriverOmegaAxis() {
    //return -xboxController.getLeftStickOmega();
    return -xboxDriveController.getLeftStickX() * 0.6;
  }


}
