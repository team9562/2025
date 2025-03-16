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

    // Update vision data
    m_visionSubsystem.findBestCameraToTarget();
    myBestCamera = m_visionSubsystem.getBestCamera();
    myClosestTarget = m_visionSubsystem.getClosestTarget();

    // If no valid target, STOP IMMEDIATELY
    if (myClosestTarget == null || myBestCamera == null) {
        System.out.println("[WARN] No valid target or camera detected. Stopping movement.");
        
        this.m_drivetrain.setControl(m_drive.withRotationalRate(0));
        this.m_drivetrain.setControl(m_drive.withVelocityX(0).withVelocityY(0)); // Stop all movement
        return; // Exit early
    }

    try {
        currentYaw = myClosestTarget.getYaw();
        System.out.println("[GO-TO] Got target YAW: " + currentYaw);
    } catch (Exception e) {
        System.out.println("[ERROR] closestTarget.getYaw() failed! " + e.getMessage());

        // Stop movement if yaw cannot be retrieved
        this.m_drivetrain.setControl(m_drive.withRotationalRate(0));
        this.m_drivetrain.setControl(m_drive.withVelocityX(0).withVelocityY(0));
        return;
    }

    // Determine movement direction
    double yawDirection;
    double accuracyYaw = 5; // Acceptable margin of error for yaw
    if (currentYaw > accuracyYaw) { 
        yawDirection = (m_visionSubsystem.compareCameras(myBestCamera) == 0) ? -1 : 1; // Adjust left/right based on camera
    } else if (currentYaw < -accuracyYaw) { 
        yawDirection = (m_visionSubsystem.compareCameras(myBestCamera) == 0) ? 1 : -1;
    } else { 
        yawDirection = 0; // Target reached
    }

    System.out.println("[GO-TO INFO] Tag detected! Yaw: " + currentYaw + " | Moving: " + yawDirection);

    // **Move Robot Sideways**  
    this.m_drivetrain.setControl(m_drive.withVelocityY(yawDirection * 0.5));
}


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // if(currentYaw < accuracyYaw && distance < minDistance)
  }
}
