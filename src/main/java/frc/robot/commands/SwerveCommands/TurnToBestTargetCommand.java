// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveCommands;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import static edu.wpi.first.units.Units.*;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurnToBestTargetCommand extends Command {

  private final CommandSwerveDrivetrain m_drivetrain;
  private final SwerveRequest.FieldCentric m_drive;
  private PhotonTrackedTarget closestTarget;
  // private Pigeon2 imu;
  // private double target; // to track a specific tag number in the future (not
  // yet implemented)
  private int myCamNum;
  private double currentYaw;
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
  private final VisionSubsystem m_visionSubsystem; // for A.T follow command
  private double adjustedYaw;
  private double speedToTarget = 1;
  private boolean isDoneAdjusting = false;
  private static CommandXboxController XController = new CommandXboxController(0);


  public TurnToBestTargetCommand(CommandSwerveDrivetrain sub1, VisionSubsystem sub2, FieldCentric request, int camNum) {
    this.m_drivetrain = sub1;
    this.m_visionSubsystem = sub2;
    this.m_drive = request;
    this.myCamNum = camNum;
    // imu = m_drivetrain.getPigeon2();

    addRequirements(m_drivetrain);
    addRequirements(m_visionSubsystem);
    // addRequirements(m_drive); // mb not a subsystem
  }

  // add the private methods here
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // whadda skibidi do i do here brah (black magic ahh)
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // XController.b().onTrue(); // reset exit state

    closestTarget = m_visionSubsystem.getBestTarget(myCamNum); // Get the closest AprilTag target from the camera

    if (closestTarget == null) {
      System.out.println("[WARN] No valid target detected. Stopping rotation.");
      this.m_drivetrain.setControl(m_drive.withRotationalRate(0));
      return; // Exit early to prevent null pointer issues
    }

    try {
      currentYaw = closestTarget.getYaw(); // Attempt to get yaw
    } catch (Exception e) {
      System.out.println("[ERROR] closestTarget.getYaw() failed! " + e.getMessage());
      this.m_drivetrain.setControl(m_drive.withRotationalRate(0));
      return;
    }

    double yawDirection;
    adjustedYaw = currentYaw;
    if(myCamNum == 1){ // adjusting for the angle they face
      adjustedYaw = currentYaw - 22.5;
    }else if(myCamNum == 2){
      adjustedYaw = currentYaw + 22.5;
    }



  double decreasingFactor = 0; // Math.abs(adjustedYaw) * 0.01; // Gradually reduce speed near target -> fix later
  double accuracyYaw = 5; // how far off the yaw can be from perfect center
    if (adjustedYaw > 3) { // If target is to the right of midpoint
      yawDirection = 1; // Turn left
      speedToTarget = 0.05 * Math.abs(adjustedYaw);
    } else if (adjustedYaw < -3) { // If target is to the left of midpoint
      yawDirection = -1; // Turn right
      speedToTarget = 0.05 * Math.abs(adjustedYaw);
    } else { // The target yaw is between +/- accuracyYaw
      yawDirection = 0; // Stop rotating when aligned
      accuracyYaw--;
      speedToTarget = 0.15;
      if(adjustedYaw < 1 && adjustedYaw > -1){
         isDoneAdjusting = true; // exit command
      }
    }

    System.out.println("[INFO] Tag detected! Yaw: " + adjustedYaw + " | Turning: " + yawDirection);
    if(speedToTarget < 0.5){
    this.m_drivetrain.setControl(m_drive.withVelocityY(yawDirection * speedToTarget)); // speedToTarget
    } else{
      this.m_drivetrain.setControl(m_drive.withVelocityY(yawDirection * 0.75));
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // should not end unless button is toggled / lifted
    return false; // isDoneAdjusting
  }
}
