// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetAngleCommand extends Command {
  /** Creates a new setAngle. */

  private final CommandSwerveDrivetrain m_drivetrain;
  private final FieldCentric m_drive;
  private final double m_angularRate;
  private final CommandJoystick m_yoke;
  private Pigeon2 imu;
  private Boolean xTrue;
  private double target;
  private double currentYaw;

  public SetAngleCommand(CommandSwerveDrivetrain subsystem, FieldCentric request, double MaxAngularRate,
      CommandJoystick yoke) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drivetrain = subsystem;
    this.m_drive = request;
    this.m_angularRate = MaxAngularRate;
    this.m_yoke = yoke;
    imu = m_drivetrain.getPigeon2();
    addRequirements(m_drivetrain);
  }

  private double getPigeonYaw() {
    return Math.abs(Math.round(imu.getAngle() % 360));
  }

  public void outputYawTarget() {
    System.out.println("Yaw: " + currentYaw);
    System.out.println("Target: " + target);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    double thumbpadX = m_yoke.getRawAxis(5);
    System.out.println(thumbpadX);

    double thumbpadY = m_yoke.getRawAxis(6);
    System.out.println(thumbpadY);

    xTrue = Math.abs(thumbpadX) > Math.abs(thumbpadY) ? true : false;

    if (xTrue) {
      if (thumbpadX > 0) {
        target = 0;
      } else if (thumbpadX < 0) {
        target = 180;
      }
    }

    if (!xTrue) {
      if (thumbpadY > 0) {
        target = 90;
      }

      else if (thumbpadY < 0) {
        target = 270;
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
