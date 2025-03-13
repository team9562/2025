// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.utils.Utility;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HomeElevator extends Command {
  ElevatorSubsystem m_ElevatorSubsystem;
  boolean zeroOrHome;

  /** Creates a new HomeToCoralStation. */
  public HomeElevator(ElevatorSubsystem sub1) {
    m_ElevatorSubsystem = sub1;
    addRequirements(m_ElevatorSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    zeroOrHome = Utility.withinTolerance(m_ElevatorSubsystem.getEncoderPose(), 0, 3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (zeroOrHome) {
      m_ElevatorSubsystem.runCurrentZeroing();
    }

    if (!zeroOrHome) {
      m_ElevatorSubsystem.setElevatorHeight(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ElevatorSubsystem.stopElevator();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_ElevatorSubsystem.isAtTarget() || Math.floor(m_ElevatorSubsystem.getEncoderPose()) == 0;
  }
}
