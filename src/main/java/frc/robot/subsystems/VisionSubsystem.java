// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

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
  private PhotonCamera[] cameras = {camera1, camera2, camera3, camera4};

  private PhotonPipelineResult[] results = new PhotonPipelineResult[4];

  private Transform3d camera1ToRobot;
  private Transform3d camera2ToRobot;
  private Transform3d camera3ToRobot;
  private Transform3d camera4ToRobot;
  private final Transform3d[] cameraPositions = {camera1ToRobot, camera2ToRobot, camera3ToRobot, camera4ToRobot};

  private static final Map<PhotonCamera, Double> CAMERA_HEIGHTS = new HashMap<>();
  private static final Map<PhotonCamera, Double> CAMERA_PITCHES = new HashMap<>();

  PhotonTrackedTarget closestTarget = null;
  PhotonTrackedTarget bestCamera = null;

  Pose3d robotPose;

  private static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout
  .loadField(AprilTagFields.k2025Reefscape);

  public VisionSubsystem() {
    this.cameraPositions[0] = VisionConstants.camera1ToRobot;
    this.cameraPositions[1] = VisionConstants.camera2ToRobot;
    this.cameraPositions[2] = VisionConstants.camera3ToRobot;
    this.cameraPositions[3] = VisionConstants.camera4ToRobot;

    this.camera1 = new PhotonCamera(VisionConstants.cameraName1);
    this.camera2 = new PhotonCamera(VisionConstants.cameraName2);
    this.camera3 = new PhotonCamera(VisionConstants.cameraName3);
    this.camera4 = new PhotonCamera(VisionConstants.cameraName4);

    CAMERA_HEIGHTS.put(camera1, VisionConstants.camera1Z);
    CAMERA_HEIGHTS.put(camera2, VisionConstants.camera2Z);
    CAMERA_HEIGHTS.put(camera3, VisionConstants.camera3Z);
    CAMERA_HEIGHTS.put(camera4, VisionConstants.camera4Z);

    CAMERA_PITCHES.put(camera1, VisionConstants.camera1pitch);
    CAMERA_PITCHES.put(camera2, VisionConstants.camera2pitch);
    CAMERA_PITCHES.put(camera3, VisionConstants.camera3pitch);
    CAMERA_PITCHES.put(camera3, VisionConstants.camera4pitch);
  }

  public void findBestCameraToTarget(){
    for(PhotonCamera camera : cameras){
      PhotonPipelineResult result = camera.getLatestResult();

      if(result.hasTargets()){
        List<PhotonTrackedTarget> targets = result.getTargets();

        double CamHeight = CAMERA_HEIGHTS.get(camera);
        double CamPitch = CAMERA_PITCHES.get(camera);

        for (PhotonTrackedTarget target : targets){
          double distance = PhotonUtils.calculateDistanceToTargetMeters(CamHeight, 0, 0, 0);
        }
      }
    }
  }

  public PhotonTrackedTarget getBestTarget(int cameraNum){
    results[0] = camera1.getLatestResult();
    results[1] = camera2.getLatestResult();
    results[2] = camera3.getLatestResult();
    results[3] = camera4.getLatestResult();
    return results[cameraNum].getBestTarget();
  }

  public Pose2d estimatePose(int i) {
    if (aprilTagFieldLayout.getTagPose(getBestTarget(i).getFiducialId()).isPresent()) {
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
  }
}
