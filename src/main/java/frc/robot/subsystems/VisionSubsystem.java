// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  private PhotonCamera camera1;
  private PhotonCamera camera2;
  private PhotonCamera camera3;
  private PhotonCamera camera4;

  private static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

  private Transform3d camera1ToRobot;
  private Transform3d camera2ToRobot;
  private Transform3d camera3ToRobot;
  private Transform3d camera4ToRobot;
  private final Transform3d[] cameraPositions = {camera1ToRobot, camera2ToRobot, camera3ToRobot, camera4ToRobot};

  Pose3d robotPose;

  List<PhotonTrackedTarget> totalTargets;
  PhotonTrackedTarget trueTarget;
  int targetID;
  boolean hasTargets;

  PhotonPipelineResult result1;
  PhotonPipelineResult result2;
  PhotonPipelineResult result3;
  PhotonPipelineResult result4;
  PhotonPipelineResult[] allResults = {result1, result2, result3, result4};

  public VisionSubsystem() {
    this.cameraPositions[0] = VisionConstants.camera1ToRobot;
    this.cameraPositions[1] = VisionConstants.camera2ToRobot;
    this.cameraPositions[2] = VisionConstants.camera3ToRobot;
    this.cameraPositions[3] = VisionConstants.camera4ToRobot;

    this.camera1 = new PhotonCamera(VisionConstants.cameraName1);
    this.camera2 = new PhotonCamera(VisionConstants.cameraName2);
    this.camera3 = new PhotonCamera(VisionConstants.cameraName3);
    this.camera4 = new PhotonCamera(VisionConstants.cameraName4);
  }

  public PhotonTrackedTarget getBestTarget(int i){
    return allResults[i].getBestTarget();
  }

  public Pose2d estimatePose(int i){
    if(aprilTagFieldLayout.getTagPose(getBestTarget(i).getFiducialId()).isPresent()){
      robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
        getBestTarget(i).getBestCameraToTarget(), 
        aprilTagFieldLayout.getTagPose(getBestTarget(i).getFiducialId()).get(), 
        cameraPositions[i]);
    }
    return robotPose.toPose2d();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    allResults[0] = camera1.getLatestResult();
    allResults[1] = camera2.getLatestResult();
    allResults[2] = camera3.getLatestResult();
    allResults[3] = camera4.getLatestResult();
    
    hasTargets = result1.hasTargets();
    totalTargets = result1.getTargets();
  }
}
