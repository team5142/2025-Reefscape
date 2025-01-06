// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EnabledSubsystems;
import frc.robot.Constants.VisionConstants.PhotonVisionConstants;

public class PhotonVisionNoteHuntingSubsystem extends SubsystemBase {

  PhotonCamera camera;
  private boolean cameraConnected = true;
  private double xAngleToNoteSaved = 0;
  private double yAngleToNoteSaved = 0;

  private double heightOfCamera = 0.232;
  //private double cameraPitchOffset = 2.519;
  private double cameraPitchOffset = 1.9;
  private double centerOfRobotToCamera = 0.28;
  private PhotonPipelineResult currentTarget;

  /** Creates a new PhotonVisionNoteHuntingSubsystem. */
  public PhotonVisionNoteHuntingSubsystem(String cameraName) {
      if (! EnabledSubsystems.noteHuntingCamera) { return; }

      try {
      camera = new PhotonCamera(cameraName); // TODO: check camera name
      cameraConnected = camera.isConnected();
      camera.setDriverMode(false);
      } catch(Exception e) {
        cameraConnected =  false;
      }

      System.out.println("Camera connected:"+camera.isConnected());
  }

  public boolean isNoteDetected() {
    return currentTarget==null && currentTarget.hasTargets();
  }

  private void updateCurrentNote(){
    try {
      if(cameraConnected){
        var result = camera.getAllUnreadResults();
        if(!result.isEmpty()){
          currentTarget = result.get(result.size()-1);
        } else {
          currentTarget = null;
        }
      } else{
        currentTarget = null;
      }
    } catch (Exception e) {
    }
  }

  public double xAngleToNote() {

    //if (!cameraConnected) {
    //  return 0;
    //}

    try {

      if (!isNoteDetected()) { // I do not see notes return 0 angle
        return 0;
      }
      PhotonTrackedTarget target = currentTarget.getBestTarget();
      return -target.getYaw(); // getYaw here returns positive right
    } catch (Exception e) {
      return 0;
    }

  }

  public double yAngleToNote() {

    //if (!cameraConnected) {
    //  return 0;
    //}

    try {

      if (!isNoteDetected()) { // I do not see notes return 0 angle
        return 0;
      }
      PhotonTrackedTarget target = currentTarget.getBestTarget();
      return target.getPitch(); 
    } catch (Exception e) {
      return 0;
    }

  }

  public void xAngleToNoteSaved() {

    //if (!cameraConnected) {
    //  return 0;
    //}

    try {

      if (!isNoteDetected()) { // I do not see notes return 0 angle
        xAngleToNoteSaved = Double.NaN;
      }
      PhotonTrackedTarget target = currentTarget.getBestTarget();
      xAngleToNoteSaved = -target.getYaw(); // getYaw here returns positive right
    } catch (Exception e) {
        xAngleToNoteSaved = Double.NaN;
    }

  }

   public void xyAngleToNoteSaved() {

    //if (!cameraConnected) {
    //  return 0;
    //}

    try {

      if (!isNoteDetected()) { // I do not see notes return 0 angle
        xAngleToNoteSaved = Double.NaN;
        yAngleToNoteSaved = Double.NaN;
        System.out.println(" Note Not Detected ");
      }
      PhotonTrackedTarget target = currentTarget.getBestTarget();
      xAngleToNoteSaved = -target.getYaw() + PhotonVisionConstants.yawOffSet; // getYaw here returns negative right
      yAngleToNoteSaved = target.getPitch();
    } catch (Exception e) {
        xAngleToNoteSaved = Double.NaN;
        yAngleToNoteSaved = Double.NaN;
        System.out.println(" Exception Detecting Note ");
    }

  }

  public double getxAngleToNoteSaved(){

    System.out.println("X : " + xAngleToNoteSaved);
    return xAngleToNoteSaved;
    
  }

  public double getyAngleToNoteSaved(){

    System.out.println("y : " + yAngleToNoteSaved);
    return yAngleToNoteSaved;
    
  }

  public void resetXAngleToNoteSaved() {
    xAngleToNoteSaved = 0;
  }

  public void resetYAngleToNoteSaved() {
    yAngleToNoteSaved = 0;
  }

  public double fromCameraToTargetSaved(){
    return fromCameraToTarget(yAngleToNoteSaved, xAngleToNoteSaved);
  }

  public double fromCameraToTarget(double pitch, double yaw) {
    double alpha = Math.toRadians(-pitch + cameraPitchOffset);
    double yawForCalculation = Math.toRadians(Math.abs(yaw));

    double targetHorizontalDistance = heightOfCamera/Math.tan(alpha);
    //System.out.println("THD: " + targetHorizontalDistance);

    return targetHorizontalDistance/Math.cos(yawForCalculation);
  }

  public double angleToTurnToNote(double yaw, double distance) {
    double alpha = Math.toRadians(180 - yaw);
    double centerOfRobotToTarget = Math.sqrt(Math.pow(centerOfRobotToCamera, 2) + Math.pow(distance, 2) - 
    2*centerOfRobotToCamera*distance*Math.cos(alpha));
    double beta = Math.toDegrees(Math.asin((Math.sin(alpha)*distance)/centerOfRobotToTarget));
    return beta;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateCurrentNote();
  }
}
