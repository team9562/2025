// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AprilTagCommands;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlignRobot extends Command {
  /** Creates a new AutoAlignRobot. */

  CommandSwerveDrivetrain m_drivetrain;
  FieldCentric m_drive;

  VisionSubsystem m_vision;

  PhotonTrackedTarget nearestTarget;
  int nearestTargetID;

  public AutoAlignRobot(CommandSwerveDrivetrain driveSub, VisionSubsystem visionSub, FieldCentric driveType) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drivetrain = driveSub;
    this.m_vision = visionSub;
    this.m_drive = driveType;

    addRequirements(m_drivetrain, m_vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    nearestTarget = m_vision.getBestTarget();
    nearestTargetID = m_vision.getTargetID();

    //find target pitch
    //find target yaw

    //calculates based on camera height, target height, camera pitch (0), target pitch (0) I think
    //PhotonUtils.calculateDistanceToTargetMeters();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //m_drivetrain.setControl(m_drive);

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
