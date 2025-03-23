// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveCommands;

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
import java.util.Timer;
import java.util.TimerTask;

import javax.swing.text.html.HTML.Tag;
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToBestTargetCommand extends Command {
  private final CommandSwerveDrivetrain m_drivetrain;
  private final SwerveRequest.FieldCentric m_drive;
  //private PhotonTrackedTarget closestTarget;
  // private double target; // to track a specific tag number in the future (not
  // yet implemented)
  //private int myidNum;
  private double currentYaw;
  private PhotonTrackedTarget myClosestTarget;
  private PhotonCamera myCamera;
  private boolean funnelCamUsed;
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
  private final VisionSubsystem m_visionSubsystem; // for A.T follow command
  private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // check final idk
  private double accuracyYaw = 5; // Acceptable margin of error or tolerance value for yaw -> initial value that shrinks when closing in on target
  private double speedToTarget = 0.5; // in meters per second -> same as above
  private boolean alignedToTarget = false; // can use this depending if we want to account for potential error or not 
  // -> using this would decrease efficiency but increase potential alignment error
  private boolean isDoneAligning = false; // for the final "isfinished" check

  private final double scoringDisplacementA = 0.2511; // A: Cam to middle of arm in meters
  private final double scoringDisplacementB = 0.1651; // B: Tag to coral branches in meters

  private Timer wallPushTimer;
  private boolean pushingIntoWall = false;

  public GoToBestTargetCommand(CommandSwerveDrivetrain sub1, VisionSubsystem sub2, FieldCentric request, boolean alfredo) { // alfredo = robot side -> funnel side = true
    this.m_drivetrain = sub1;
    this.m_visionSubsystem = sub2;
    this.m_drive = request;
    this.funnelCamUsed = alfredo;

    //this.myidNum = idNum;
    // imu = m_drivetrain.getPigeon2();

    addRequirements(m_drivetrain);
    addRequirements(m_visionSubsystem); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_visionSubsystem.findBestCameraToTarget(); // idk if need this anymore -> see l8r
    myCamera = m_visionSubsystem.cameraPicker(funnelCamUsed);
    myClosestTarget = m_visionSubsystem.getBestTarget(m_visionSubsystem.compareCameras(myCamera));
  }

  // Called every time the scheduler runs while the command is scheduled.


  @Override
public void execute() {

  System.out.println("[GOTOBESTTARGET]: COMMAND IS CALLED");

    // Update vision data

    // If no valid target, STOP IMMEDIATELY
    if (myClosestTarget == null || myCamera == null) {
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
    System.out.println("[GOTO PASSED ERRORS]: PASSED ALL ERROR CATCHES");

    double yawDirection;
    if (currentYaw > accuracyYaw) { 
        yawDirection = (m_visionSubsystem.compareCameras(myCamera) == 0) ? -1 : 1; // Adjust left/right based on camera
    } else if (currentYaw < -accuracyYaw) { 
        yawDirection = (m_visionSubsystem.compareCameras(myCamera) == 0) ? 1 : -1;
    } else { 
      yawDirection = 0;
      if(accuracyYaw >=1){ // limit the tolerance value to 1 (smallest possible error)
        accuracyYaw--; // make the error tolerance value smaller
        speedToTarget = speedToTarget/2; // divide the speed by 2 to make it easier to approach a smaller tolerance value
      }
    }
    System.out.println("[GO-TO DIRECTION] Tag detected! Yaw: " + currentYaw + " | Moving: " + yawDirection);

    // **Move Robot Sideways**  
    this.m_drivetrain.setControl(m_drive.withVelocityY(yawDirection * speedToTarget)); // -> starts at (+/- 1) * (0.5)
    System.out.println("[GOTO MOVED ALIGN]: ALIGNING TO TARGET");

    // Y-axis is for side-to-side movement

    // add two integer arrays of tag ids here to access values for scoring area displacement (x&y)
    // then access array values with getFuidicalID() and compare to determine displacement
    // run velocity command to align to specific id-camera displacement
    // finally we can make the robot go forward until distance fwd/bwd is also aligned
if(currentYaw<accuracyYaw){
    if(m_visionSubsystem.compareCameras(myCamera) == 0 && (myClosestTarget.getFiducialId() == 1 || myClosestTarget.getFiducialId() == 2)){ // coral intake station camera
      startPushingIntoWall(1, 0.2511, 0.2); // cam to middle arm displacement A -> middle arm is now aligned to middle tag
      startPushingIntoWall(1, 0.1651, 0.2); // tag to coral branches displacement B -> arm is now aligned to a coral branch column
      while(m_visionSubsystem.getTargetDistance(myClosestTarget.getFiducialId(), m_visionSubsystem.compareCameras(myCamera)) > 1){
        // drive towards coral station until distance is less than 2 meters
        this.m_drivetrain.setControl(m_drive.withVelocityX(1)); // direction +/- 1
        
        
      }
      while(m_visionSubsystem.getTargetDistance(myClosestTarget.getFiducialId(), m_visionSubsystem.compareCameras(myCamera)) > 0.75){
        // drive towards coral station until distance is less than 0.75 meters
        this.m_drivetrain.setControl(m_drive.withVelocityX(0.25)); // direction +/- 1
      }
      //this.m_drivetrain.setControl(m_drive.withVelocityX(0.2)); // direction +/- 1
      // this last one is incase any extra push into the wall is needed
      System.out.println("[GOTO CLOSED DISTANCE]: CLOSED DISTANCE");
      isDoneAligning = true;
    } else if(m_visionSubsystem.compareCameras(myCamera) != 0 && (myClosestTarget.getFiducialId() <= 11 || myClosestTarget.getFiducialId() >= 6)){ // reef tags
      while(m_visionSubsystem.getTargetDistance(myClosestTarget.getFiducialId(), m_visionSubsystem.compareCameras(myCamera)) > 1){
        // drive towards coral station until distance is less than 2 meters
        this.m_drivetrain.setControl(m_drive.withVelocityX(1)); // direction +/- 1
      }
      while(m_visionSubsystem.getTargetDistance(myClosestTarget.getFiducialId(), m_visionSubsystem.compareCameras(myCamera)) > 0.75){
        // drive towards coral station until distance is less than 0.75 meters
        this.m_drivetrain.setControl(m_drive.withVelocityX(0.25)); // direction +/- 1
      }

      startPushingIntoWall(1, 0.75, 0.25); // cam to middle arm displacement A -> middle arm is now aligned to middle tag

      //this.m_drivetrain.setControl(m_drive.withVelocityX(0.2)); // direction +/- 1
      // this last one is incase any extra push into the wall is needed

      isDoneAligning = true;
      System.out.println("[GOTO CLOSED DISTANCE]: CLOSED DISTANCE");

    } else isDoneAligning = true; // something else happened so exit by assuming that alignment is done
  }
  }


private void startPushingIntoWall(int directionBobabowa, double targetDisplacement, double speedToTarget) { // directionBobabowa = +/- 1 , msPushDuration = # in ms
  long msPushDuration = Math.round(targetDisplacement/speedToTarget); // time = displacement / velocity
  if (!pushingIntoWall) {
      pushingIntoWall = true;
      System.out.println("[ALIGN] Pushing into the wall for " + msPushDuration + "ms.");
      this.m_drivetrain.setControl(m_drive.withVelocityX(directionBobabowa*0.2)); // Small forward OR backward force depending on +/-

      // Start a timer to stop pushing after `pushDuration` milliseconds
      wallPushTimer = new Timer();
      wallPushTimer.schedule(new TimerTask() {
          @Override
          public void run() {
              stopMovement();
              isDoneAligning = true; // Mark as finished
              pushingIntoWall = false;
          }
      }, msPushDuration);
  }
}

/** Stops the robot's movement */
private void stopMovement() {
  this.m_drivetrain.setControl(m_drive.withVelocityX(0).withVelocityY(0));
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {// if(currentYaw < accuracyYaw)
    return currentYaw < accuracyYaw;
  }
}
