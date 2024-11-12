// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.VisionConstants;

/** Add your docs here. */
public interface VisionHelpers {

  public default double getDistanceToRedSpeaker(Pose2d robotPose) {
    if (robotPose == null) {  // return NaN if the camera cannot determine the pose
      return Double.NaN;
    }
    return robotPose.getTranslation().getDistance(VisionConstants.redSpeakerTranslation);
  }

  public default double getDistanceToBlueSpeaker(Pose2d robotPose) {
    if (robotPose == null) {  // return NaN if the camera cannot determine the pose
      return Double.NaN;
    }
    return robotPose.getTranslation().getDistance(VisionConstants.blueSpeakerTranslation);
  }

  public default double getDistanceToRedAmp(Pose2d robotPose) {
    if (robotPose == null) {  // return NaN if the camera cannot determine the pose
      return Double.NaN;
    }
    return robotPose.getTranslation().getDistance(VisionConstants.redAmpTranslation);
  }

  public default double getDistanceToBlueAmp(Pose2d robotPose) {
    if (robotPose == null) {  // return NaN if the camera cannot determine the pose
      return Double.NaN;
    }
    System.out.println(robotPose.getTranslation().getDistance(VisionConstants.blueAmpTranslation));
    return robotPose.getTranslation().getDistance(VisionConstants.blueAmpTranslation);
  }

  public default Rotation2d getAngleToRedSpeaker(Pose2d robotPose) {
    if (robotPose == null) {  // return NaN if the camera cannot determine the pose
      return Rotation2d.fromDegrees(0);
    }
    return robotPose.relativeTo(VisionConstants.redSpeakerPose).getRotation();
  }

  public default Rotation2d getAngleToBlueSpeaker(Pose2d robotPose) {
    if (robotPose == null) {  // return NaN if the camera cannot determine the pose
      return Rotation2d.fromDegrees(0);
    }
    return robotPose.relativeTo(VisionConstants.blueSpeakerPose).getRotation();
  }

  public default Rotation2d getAngleToRedAmp(Pose2d robotPose) {
    if (robotPose == null) {  // return NaN if the camera cannot determine the pose
      return null;
    }
    return robotPose.relativeTo(VisionConstants.redAmpPose).getRotation();
  }

  public default Rotation2d getAngleToBlueAmp(Pose2d robotPose) {
    if (robotPose == null) {  // return NaN if the camera cannot determine the pose
      return null;
    }
    return robotPose.relativeTo(VisionConstants.blueAmpPose).getRotation();
  }

}
