// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveCommands;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import static edu.wpi.first.units.Units.*;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class OffsetAlignment extends Command {
  /** Creates a new OffsetAlignment. */
  private final CommandSwerveDrivetrain m_drivetrain;
  private final SwerveRequest.RobotCentric m_drive = new RobotCentric();
  private PhotonTrackedTarget closestTarget;
  private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private boolean isLeftBranch;
  private int yawDirection;
  private final double scoringDisplacementA = 0.2511; // A: Cam to middle of arm in meters
  private final double scoringDisplacementB = 0.1651; // B: Tag to coral branches in meters


  public OffsetAlignment(CommandSwerveDrivetrain sub1, boolean leftBranch) {
    this.m_drivetrain = sub1;
    this.isLeftBranch = leftBranch;

    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(isLeftBranch){
      yawDirection = -1;
    } else{
      yawDirection = 1;
    }
    driveThisWay().withTimeout(0.51);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  private Command driveThisWay(){
    return m_drivetrain.run(() -> m_drivetrain.setControl(m_drive.withVelocityY(yawDirection * 1.02)));
  }
}
