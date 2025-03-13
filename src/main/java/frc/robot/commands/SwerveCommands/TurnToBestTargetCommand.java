// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveCommands;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import static edu.wpi.first.units.Units.*;
import org.photonvision.targeting.PhotonTrackedTarget;

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
    double decreasingFactor = Math.abs(currentYaw) * 0.01; // Gradually reduce speed near target
double accuracyYaw = 0.1; // 
    if (currentYaw > accuracyYaw) { // If target is to the right of midpoint
      yawDirection = -1; // Turn left
    } else if (currentYaw < -accuracyYaw) { // If target is to the left of midpoint
      yawDirection = 1; // Turn right
    } else { // The target yaw is between -0.1 and 0.1 (-0.1 < yaw < 0.1)
      yawDirection = 0; // Stop rotating when aligned
    }

    System.out.println("[INFO] Tag detected! Yaw: " + currentYaw + " | Turning: " + yawDirection);

    this.m_drivetrain.setControl(m_drive.withRotationalRate((yawDirection * MaxAngularRate/4) * (1 - decreasingFactor)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // should not end unless button is toggled / lifted
    return false;
  }
}
