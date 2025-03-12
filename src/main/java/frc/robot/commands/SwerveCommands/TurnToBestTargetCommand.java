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

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurnToBestTargetCommand extends Command {

  private final CommandSwerveDrivetrain m_drivetrain;
  private final SwerveRequest.FieldCentric m_drive;
  private Pigeon2 imu;
  private double target;
  private double currentYaw;
  private double m_angularRate;
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem(); // for A.T follow command

  public TurnToBestTargetCommand(CommandSwerveDrivetrain subsystem, FieldCentric request, double angularRate) {
    this.m_drivetrain = subsystem;
    this.m_drive = request;
    this.m_angularRate = angularRate;
    imu = m_drivetrain.getPigeon2();

    addRequirements(m_drivetrain);

  }

  // add the private methods here

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { // !!!!!!!!!!!!!!!! if the camera is inverted (180) we might have to switch the +/- values for the logic to work

    currentYaw = m_visionSubsystem.getBestYaw(); // get the yaw of the closest target
    double yawDirection;
    double decreasingFactor = 0; //add some sort of decreasing factor for the angular rate to gradually decrease once it approaches yaw = 0
    if(currentYaw>0.1){
      yawDirection = -1;
    }
    else if(currentYaw < -0.1){
      yawDirection = 1;
    } else {
      yawDirection = 0;
    }
    this.m_drivetrain.setControl(m_drive.withRotationalRate((yawDirection * m_angularRate)-decreasingFactor)); 


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
