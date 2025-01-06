// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;


import frc.robot.RobotContainer;

import frc.robot.Constants.SwerveConstants.SwerveChassis;

/** Add your docs here. */
public class AutonomousTrajectoryRioCommand extends FollowPathCommand {

    public AutonomousTrajectoryRioCommand(PathPlannerPath trajectoryPath, RobotConfig robotConfig)  {
        super(
            trajectoryPath, // PathPlannerPath
            RobotContainer.driveSubsystem::getPose,  // Supplier<Pose2d>
            RobotContainer.driveSubsystem::getChassisSpeeds, // Supplier<ChassisSpeeds>
            RobotContainer.driveSubsystem::driveWithChassisSpeeds, // BiConsumer<ChassisSpeeds,DriveFeedforwards>
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(
                        SwerveChassis.DRIVE_CHASSIS_KP,
                        SwerveChassis.DRIVE_CHASSIS_KI,
                        SwerveChassis.DRIVE_CHASSIS_KD), // Translation PID constants
                    new PIDConstants(
                        SwerveChassis.ANGLE_CHASSIS_KP, 
                        SwerveChassis.ANGLE_CHASSIS_KI,
                        SwerveChassis.ANGLE_CHASSIS_KD
                    ) // Rotation PID constants
            ),
            /* 
            new RobotConfig(
                    SwerveChassis.robotMass, // kg
                    SwerveChassis.robotInertia, // kg*m^2 for rotation
                    new ModuleConfig(
                        SwerveChassis.WHEEL_DIAMETER/2.0, // wheel radius
                        SwerveChassis.MaxSpeed, // Max module speed, in m/s
                        SwerveChassis.wheelCOF, // wheel COF
                        DCMotor.getFalcon500(1),
                        CurrentLimiter.drive, // type of motor used
                        1
                    ),

                    new Translation2d(SwerveChassis.WHEEL_BASE / 2.0,SwerveChassis.TRACK_WIDTH / 2.0),
                    new Translation2d(SwerveChassis.WHEEL_BASE / 2.0,-SwerveChassis.TRACK_WIDTH / 2.0),
                    new Translation2d(-SwerveChassis.WHEEL_BASE / 2.0,SwerveChassis.TRACK_WIDTH / 2.0),
                    new Translation2d(-SwerveChassis.WHEEL_BASE / 2.0,-SwerveChassis.TRACK_WIDTH / 2.0)
            ),
            */
            robotConfig,
            ()->{return false;}, // shouldFlipPath - only on red side
            RobotContainer.driveSubsystem // subsystem requirement
        );
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        super.initialize();
        System.out.println("Auto trajectory initialized");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        super.execute();
        System.out.println("cp: " + RobotContainer.driveSubsystem.getPose());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        System.out.println("*** End trajectory command. Interrupted:" + interrupted);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        return super.isFinished();

    }

}
