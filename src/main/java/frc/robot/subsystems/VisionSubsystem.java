// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  private final PhotonCamera camera1 = new PhotonCamera(VisionConstants.cameraName1);

  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

  Transform3d cameraToRobot;

  Pose3d robotPose;

  List<PhotonTrackedTarget> totalTargets;
  PhotonTrackedTarget trueTarget;
  int targetID;
  boolean hasTargets;

  PhotonPipelineResult result1;

  public VisionSubsystem() {
    cameraToRobot = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));
  }

  public PhotonTrackedTarget getBestTarget(){
    return result1.getBestTarget();
  }

  public int getTargetID(){
    return getBestTarget().getFiducialId();
  }

  public Pose3d estimatePose(){
    if(aprilTagFieldLayout.getTagPose(getBestTarget().getFiducialId()).isPresent()){
      robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
        getBestTarget().getBestCameraToTarget(), 
        aprilTagFieldLayout.getTagPose(getBestTarget().getFiducialId()).get(), 
        cameraToRobot);
    }

    return robotPose;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    result1 = camera1.getLatestResult();
    
    hasTargets = result1.hasTargets();

    totalTargets = result1.getTargets();
  }
}
