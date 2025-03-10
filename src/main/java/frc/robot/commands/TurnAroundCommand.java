// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurnAroundCommand extends Command {

  private final CommandSwerveDrivetrain m_drivetrain;
  private final SwerveRequest.FieldCentric m_drive;
  private Pigeon2 imu;
  private double target;
  private double currentYaw;
  private double m_angularRate;

  public TurnAroundCommand(CommandSwerveDrivetrain subsystem, FieldCentric request, double angularRate) {

    this.m_drivetrain = subsystem;
    this.m_drive = request;
    this.m_angularRate = angularRate;
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

    // if it should subtract 180 or add 180 to the value
    // imu.setYaw(0);
    currentYaw = getPigeonYaw();
    target = currentYaw < 180 ? currentYaw + 180 : currentYaw - 180;
    target = Math.round(target / 10) * 10;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // sets the wheel direction
    this.m_drivetrain.setControl(m_drive.withRotationalRate(1 * m_angularRate));

    this.currentYaw = getPigeonYaw();

    // debug info
    System.out.println("command is executing");
    outputYawTarget();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("turnAround complete");
    System.out.println("FINAL VALUES: ");
    outputYawTarget();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // stops when the yaw is at the same rotation as the target
    return Math.round(currentYaw / 10) * 10 == target - 30;
  }
}
