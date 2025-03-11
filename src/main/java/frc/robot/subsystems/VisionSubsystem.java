// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  private PhotonCamera camera1;
  private PhotonCamera camera2;
  private PhotonCamera camera3;
  private PhotonCamera camera4;
  private PhotonCamera[] cameras;

  private PhotonPipelineResult[] results;

  private final Transform3d[] cameraPositions = new Transform3d[4];

  private static final Map<PhotonCamera, Double> CAMERA_HEIGHTS = new HashMap<>();
  private static final Map<PhotonCamera, Double> CAMERA_PITCHES = new HashMap<>();
  private static Set<Integer> ID_SET;

  PhotonTrackedTarget closestTarget = null;
  PhotonCamera bestCamera = null;
  double targetHeight;
  double distance;
  double newDist = Double.MAX_VALUE;

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
    this.cameras = new PhotonCamera[] { camera1, camera2, camera3, camera4 };

    this.results = new PhotonPipelineResult[] {
        camera1.getLatestResult(),
        camera2.getLatestResult(),
        camera3.getLatestResult(),
        camera4.getLatestResult() };

    CAMERA_HEIGHTS.put(camera1, VisionConstants.camera1Z);
    CAMERA_HEIGHTS.put(camera2, VisionConstants.camera2Z);
    CAMERA_HEIGHTS.put(camera3, VisionConstants.camera3Z);
    CAMERA_HEIGHTS.put(camera4, VisionConstants.camera4Z);

    CAMERA_PITCHES.put(camera1, VisionConstants.camera1pitch);
    CAMERA_PITCHES.put(camera2, VisionConstants.camera2pitch);
    CAMERA_PITCHES.put(camera3, VisionConstants.camera3pitch);
    CAMERA_PITCHES.put(camera4, VisionConstants.camera4pitch);
  }

  public void findBestCameraToTarget() {
    for (PhotonCamera camera : cameras) {
      PhotonPipelineResult result = camera.getLatestResult();

      if (result.hasTargets()) {
        List<PhotonTrackedTarget> targets = result.getTargets();

        double CamHeight = CAMERA_HEIGHTS.get(camera);
        double CamPitch = CAMERA_PITCHES.get(camera);

        for (PhotonTrackedTarget target : targets) {

          distance = PhotonUtils.calculateDistanceToTargetMeters(CamHeight,
              aprilTagFieldLayout.getTagPose(target.getFiducialId()).get().getZ(), CamPitch, target.getPitch());

          if (distance < newDist) {
            newDist = distance;
            closestTarget = target;
            bestCamera = camera;
          }
        }
      }
    }
  }

  public void findBestCameraToPOI(String tagType) {

    if (tagType.toLowerCase().equals("coral")) {
      targetHeight = VisionConstants.CORAL_STATION_TAG;
      ID_SET = VisionConstants.Coral_ID;
    }

    if (tagType.toLowerCase().equals("processor")) {
      targetHeight = VisionConstants.PROCESSOR_TAG;
      ID_SET = VisionConstants.Processor_ID;
    }

    if (tagType.toLowerCase().equals("reef")) {
      targetHeight = VisionConstants.REEF_TAG;
      ID_SET = VisionConstants.Reef_ID;
    }

    if (tagType.toLowerCase().equals("barge")) {
      targetHeight = VisionConstants.BARGE_TAG;
      ID_SET = VisionConstants.Barge_ID;
    }

    for (PhotonCamera camera : cameras) {
      PhotonPipelineResult result = camera.getLatestResult();

      if (result.hasTargets()) {
        List<PhotonTrackedTarget> targets = result.getTargets();

        double CamHeight = CAMERA_HEIGHTS.get(camera);
        double CamPitch = CAMERA_PITCHES.get(camera);

        for (PhotonTrackedTarget target : targets) {

          if (ID_SET.contains(target.getFiducialId())) {

            distance = PhotonUtils.calculateDistanceToTargetMeters(CamHeight, targetHeight, CamPitch,
                target.getPitch());

            if (distance < newDist) {
              newDist = distance;
              closestTarget = target;
              bestCamera = camera;
            }
          }
        }
      }
    }
  }

  public PhotonTrackedTarget getBestTarget(int cameraNum) {
    results[cameraNum] = cameras[cameraNum].getLatestResult();
    if (results[cameraNum] != null && results[cameraNum].hasTargets()) {
      System.out.println("--------------------- LINE 160 - " + results[cameraNum].getBestTarget());
      return results[cameraNum].getBestTarget();
    }
    else return null;
  }

  public Pose2d estimatePose(int i, Pose2d oldPose) {
    if (getBestTarget(i) != null) {
      if (aprilTagFieldLayout.getTagPose(getBestTarget(i).getFiducialId()).isPresent()) {
        robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
            getBestTarget(i).getBestCameraToTarget(),
            aprilTagFieldLayout.getTagPose(getBestTarget(i).getFiducialId()).get(),
            cameraPositions[i]);
        return robotPose.toPose2d();
      }
      else
        return oldPose;
    }

    else
      return oldPose;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (closestTarget != null && bestCamera != null) {
      System.out.println("SOMETHING IS FOUND!!!!!!!!!");
      SmartDashboard.putString("Using Camera: ", bestCamera.getName().toString());
      SmartDashboard.putNumber("Best Yaw: ", closestTarget.getYaw());
      SmartDashboard.putNumber("Best Pitch: ", closestTarget.getPitch());
      SmartDashboard.putNumber("Best Area: ", closestTarget.getArea());
      SmartDashboard.putNumber("Exact Distance (meters): ", newDist);
    }
  }
}
