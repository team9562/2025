// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.utils.Utility;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetHeightToPOI extends Command {

  ElevatorSubsystem m_elevator;
  CommandXboxController xbox;
  double setpoint;
  double angle;

  /** Creates a new score. */
  public SetHeightToPOI(ElevatorSubsystem sub1, CommandXboxController xInput) {
    this.m_elevator = sub1;
    this.xbox = xInput;
    addRequirements(m_elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (Utility.getLatestXInput(xbox)) {
      default:
        setpoint = m_elevator.getEncoderPose();

      case "x":
        setpoint = ElevatorConstants.L2;

      case "y":
        setpoint = ElevatorConstants.L3;

      case "a":
        setpoint = ElevatorConstants.L4;

      case "b":
        setpoint = ElevatorConstants.B;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevator.setElevatorHeight(setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevator.isAtTarget();
  }
}
