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
public class GoToBestTargetCommand extends Command {
  private final CommandSwerveDrivetrain m_drivetrain;
  private final SwerveRequest.FieldCentric m_drive;
  private PhotonTrackedTarget closestTarget;
  // private Pigeon2 imu;
  // private double target; // to track a specific tag number in the future (not
  // yet implemented)
  private int myidNum;
  private double currentYaw;
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
  private final VisionSubsystem m_visionSubsystem; // for A.T follow command

  public GoToBestTargetCommand(CommandSwerveDrivetrain sub1, VisionSubsystem sub2, FieldCentric request, int idNum) {
    this.m_drivetrain = sub1;
    this.m_visionSubsystem = sub2;
    this.m_drive = request;
    this.myidNum = idNum;
    // imu = m_drivetrain.getPigeon2();

    addRequirements(m_drivetrain);
    addRequirements(m_visionSubsystem);  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
/* 
    m_drivetrain.setControl(() -> drive // setControl instead
    .withVelocityX(XController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
    .withVelocityY(XController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
    .withRotationalRate(XController.getRightY() * MaxAngularRate));
*/
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
