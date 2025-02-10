// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants.autoPoses;
import frc.robot.Constants.VisionConstants.LimeLightConstants;
import frc.robot.lib.LimelightHelpers;
import frc.robot.lib.TrajectoryHelpers;
import frc.robot.lib.VisionHelpers;

public class LLVisionSubsystem extends SubsystemBase implements VisionHelpers {
  public static double joystickDirectionDegrees;
  public static Pose2d robotCurrentPoseBeforeClimb;
  public static double distanceToShoot = -1;
  private static double llDirectionCorrection = 5.5; // LL angle correction because LL is not pointing straight

  /** Creates a new LLVisionSubsystem. 
   * AprilTag Vision Subsystem based on LimeLight.
   * The coordinate tracking is always done with 0,0 on the lower blue side of the field.
  */

/*

April Tag Coordinates

ID X       Y        Z       Z-Rot  Y-Rot
1 657.37   25.80    58.50   126    0
2 657.37   291.20   58.50   234    0
3 455.15   317.15   51.25   270    0
4 365.20   241.64   73.54   0      30
5 365.20   75.39    73.54   0      30
6 530.49   130.17   12.13   300    0
7 546.87   158.50   12.13   0      0
8 530.49   186.83   12.13   60     0
9 497.77   186.83   12.13   120    0
10 481.39  158.50   12.13   180    0
11 497.77  130.17   12.13   240    0
12 33.51   25.80    58.50   54     0
13 33.51   291.20   58.50   306    0
14 325.68  241.64   73.54   180    30
15 325.68  75.39    73.54   180    30
16 235.73  -0.15    51.25   90     0
17 160.39  130.17   12.13   240    0
18 144.00  158.50   12.13   180    0
19 160.39  186.83   12.13   120    0
20 193.10  186.83   12.13   60     0
21 209.49  158.50   12.13   0      0
22 193.10  130.17   12.13   300    0
*/

  public LLVisionSubsystem() {}

  public Pose2d getRobotFieldPoseLL() {
    if (LimelightHelpers.getTV(LimeLightConstants.LLAprilTagName)) { // LL target visible - meaning - see an Apriltag
      return LimelightHelpers.getBotPose2d_wpiBlue(LimeLightConstants.LLAprilTagName).transformBy(LimeLightConstants.cameraToRobotTransform); //TODO: Check if the coordinates need to be translated to 0,0 of the blue lower corner
      // return LimelightHelpers.getBotPose2d_wpiBlue(LimeLightConstants.LLAprilTagName).relativeTo(LimeLightConstants.centerFieldPose); // check if this returns the right pose from 0,0
    } else {
      return null; //TODO: Consider changing this class to return Optional<Pose2d>
    }
  }

  public boolean isApriltagVisible() {
    return LimelightHelpers.getTV(LimeLightConstants.LLAprilTagName);
  }

  public boolean isApriltagClimbVisible() {
    if (isApriltagVisible()) {
      // save the robot pose if we see LL
      robotCurrentPoseBeforeClimb = LimelightHelpers.getBotPose2d_wpiBlue(LimeLightConstants.LLAprilTagName);
      //.transformBy(LimeLightConstants.cameraToRobotTransform);
      double fid = getFiducialId();
      return fid>=11.0 && fid <=16.0;
    }
    return false; // if no apriltag visible
  }

  public double getShootingDistance(Pose2d pose) {
    if (RobotContainer.isAllianceRed) {

      // alex test
      //System.out.println("RP:"+pose);
      return autoPoses.RED_SPEAKER_TAG.getPose().getTranslation().getDistance(
         pose.getTranslation()
      );
    } else {
        return autoPoses.BLUE_SPEAKER_TAG.getPose().getTranslation().getDistance(
         pose.getTranslation()
      );
    }
  }

  public double getShootingDistance() {
    if (isApriltagVisible()) {
      distanceToShoot = Math.abs(getShootingDistance(getRobotFieldPoseLL())); // use absolute distance, so works on both RED and BLUE
    } else {
      distanceToShoot = -1;
    }

    // alex test
    //System.out.println("ATV0-D:"+distanceToShoot);
    return distanceToShoot;
  }

  public Pose2d getRobotFieldPoseLLBeforeClimb() {
    return robotCurrentPoseBeforeClimb;
  }

  public double getFiducialId() {
    return LimelightHelpers.getFiducialID(LimeLightConstants.LLAprilTagName);
  }

  // Rotate to point to the center of the speaker, considering the al;liance color
  public Rotation2d getRotationAngleToSpeaker() {
    if (isApriltagVisible()) {
    return TrajectoryHelpers.rotateToPointToSecondPose(
      // First parameter - current robot pose (center of the robot)
      getRobotFieldPoseLL().plus
        (new Transform2d
          (new Translation2d(0.28, 0.25), 
            Rotation2d.fromDegrees(180)
            )
          ) ,
        // Second parameter - pose to point to
        (RobotContainer.isAllianceRed)? autoPoses.RED_SPEAKER_TAG.getPose():autoPoses.BLUE_SPEAKER_TAG.getPose()
      ).plus(Rotation2d.fromDegrees(llDirectionCorrection))
      ;}
      else { // if you do not see it, do not rotate
        return Rotation2d.fromDegrees(0);
      }
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
