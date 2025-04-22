// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveCommands;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import static edu.wpi.first.units.Units.*;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RangeAlignment extends Command {
  /** Creates a new ForwardToBestTagCommand. */
  private final CommandSwerveDrivetrain m_drivetrain;
  private final SwerveRequest.FieldCentric m_drive;
  private final VisionSubsystem m_visionsubsystem;
  //private PhotonTrackedTarget closestTarget;
  // private Pigeon2 imu;
  // private double target; // to track a specific tag number in the future (not
  // yet implemented)
  //private int myidNum;
  private double currentYaw;
  private PhotonTrackedTarget myClosestTarget;
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
  private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // check final idk
  private double accuracyYaw = 5; // Acceptable margin of error for yaw
  private double adjustmentSpeed = 0.5; // m/s
  private double adjustedYaw;
  private double targetDistanceToTag;
  private double distanceToTag;
  private final int tagID;
private Pose2d prevPose2d = null;
private Pose2d newpPose2d = null;
  private final double scoringDisplacementA = 0.2511; // A: Cam to middle of arm in meters
  private final double scoringDisplacementB = 0.1651; // B: Tag to coral branches in meters
  private Pigeon2 pigeon2thing;
  public RangeAlignment(CommandSwerveDrivetrain sub1, VisionSubsystem sub2, FieldCentric request, int thisTagID) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drivetrain = sub1;
    this.m_drive = request;
    this.m_visionsubsystem = sub2;
    this.tagID = thisTagID;
    pigeon2thing = m_drivetrain.getPigeon2();

    addRequirements(m_drivetrain);
    addRequirements(m_visionsubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
    //if()

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return distanceToTag < targetDistanceToTag;
  }

  private Command driveThisWay(){
    return m_drivetrain.run(() -> m_drivetrain.setControl(m_drive.withVelocityY(1.0)));
  }
}
