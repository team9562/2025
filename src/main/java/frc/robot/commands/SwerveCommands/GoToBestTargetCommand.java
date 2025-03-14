// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveCommands;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToBestTargetCommand extends Command {
  private final CommandSwerveDrivetrain m_drivetrain;
  private final SwerveRequest.FieldCentric m_drive;
  //private PhotonTrackedTarget closestTarget;
  // private Pigeon2 imu;
  // private double target; // to track a specific tag number in the future (not
  // yet implemented)
  //private int myidNum;
  private double currentYaw;
  private PhotonTrackedTarget myClosestTarget;
  private PhotonCamera myBestCamera;
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
  private final VisionSubsystem m_visionSubsystem; // for A.T follow command
  private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // check final idk

  public GoToBestTargetCommand(CommandSwerveDrivetrain sub1, VisionSubsystem sub2, FieldCentric request) {
    this.m_drivetrain = sub1;
    this.m_visionSubsystem = sub2;
    this.m_drive = request;

    //this.myidNum = idNum;
    // imu = m_drivetrain.getPigeon2();

    addRequirements(m_drivetrain);
    addRequirements(m_visionSubsystem); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //double distance;

    //distance = PhotonUtils.calculateDistanceToTargetMeters(CamHeight, aprilTagFieldLayout.getTagPose(target.getFiducialId()).get().getZ(), CamPitch, target.getPitch());

              /* 
    m_drivetrain.setControl(() -> drive // setControl instead
    .withVelocityX(XController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
    .withVelocityY(XController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
    .withRotationalRate(XController.getRightY() * MaxAngularRate));
*/


m_visionSubsystem.findBestCameraToTarget(); // Get the closest AprilTag target from the camera
myBestCamera = m_visionSubsystem.getBestCamera();
myClosestTarget = m_visionSubsystem.getClosestTarget();

System.out.println("[GOON BOT] GoToBestTarget Command is RUNNING!!!");

if(myClosestTarget != null){
//System.out.println("[GOON] Closest tag: " + myClosestTarget.getFiducialId());
}

    if (myClosestTarget == null) {
      System.out.println("[WARN] No valid target detected. Stopping movement.");
      this.m_drivetrain.setControl(m_drive.withRotationalRate(0));
      m_drive.withVelocityX(0); 
      m_drive.withVelocityY(0); 
      return; // Exit early to prevent null pointer issues
    }

    if (myBestCamera == null) {
      System.out.println("[WARN] No valid camera detected. Stopping movement.");
      this.m_drivetrain.setControl(m_drive.withRotationalRate(0));
      m_drive.withVelocityX(0); 
      m_drive.withVelocityY(0); 
      return; // Exit early to prevent null pointer issues
    }

    try {
      currentYaw = myClosestTarget.getYaw(); // Attempt to get yaw
    } catch (Exception e) {
      System.out.println("[ERROR] closestTarget.getYaw() failed! " + e.getMessage());
      this.m_drivetrain.setControl(m_drive.withRotationalRate(0));
      m_drive.withVelocityX(0); 
      m_drive.withVelocityY(0); 

      return;
    }

    double yawDirection;
    double decreasingFactor = 0; // Math.abs(currentYaw) * 0.01; // Gradually reduce speed near target -> fix later
double accuracyYaw = 5; // how far off the yaw can be from perfect center
    if (currentYaw > accuracyYaw) { // If target is to the right of midpoint
      if((m_visionSubsystem.compareCameras(myBestCamera))==0){
        yawDirection = -1; // Go left
        } else {
          yawDirection = 1; // Go left (relative)
        }
    } else if (currentYaw < -accuracyYaw) { // If target is to the left of midpoint
      if((m_visionSubsystem.compareCameras(myBestCamera))==0){
        yawDirection = 1; // Go right
        } else {
          yawDirection = -1; // Go right (relative)
        }
      
    } else { // The target yaw is between +/- accuracyYaw, so target yaw has been reached
      yawDirection = 0; // Stop rotating when aligned
    }

    System.out.println("[INFO GO-TO] Tag detected! Yaw: " + currentYaw + " | Moving: " + yawDirection);

    //this.m_drivetrain.setControl(m_drive.withRotationalRate((yawDirection * MaxAngularRate/2.5) * (1 - decreasingFactor)));
    
    m_drivetrain.setControl(m_drive.withVelocityY(yawDirection * MaxSpeed/3)); // idk if this is the correct axis or direction -> need to test

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
