// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.utils.Utility;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HomeArm extends Command {
  ArmSubsystem m_ArmSubsystem;

  /** Creates a new HomeElevator. */
  public HomeArm(ArmSubsystem arm) {
    this.m_ArmSubsystem = arm;
    addRequirements(m_ArmSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ArmSubsystem.turnPitchMotor(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ArmSubsystem.stopPitch();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Utility.withinTolerance(m_ArmSubsystem.getEncoderPose(), 0, 1);
  }
}
