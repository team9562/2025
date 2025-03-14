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
  String poi;
  double setpoint;
  double angle;

  /** Creates a new score. */
  public SetHeightToPOI(ElevatorSubsystem sub1, String POI) {
    this.m_elevator = sub1;
    this.poi = POI;
    addRequirements(m_elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (poi.toLowerCase()) {
      default:
        setpoint = m_elevator.getEncoderPose();
        break;

      case "l2":
        setpoint = ElevatorConstants.L2;
        break;

      case "l3":
        setpoint = ElevatorConstants.L3;
        break;

      case "l4":
        setpoint = ElevatorConstants.L4;
        break;

      case "b":
        setpoint = ElevatorConstants.B;
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevator.setElevatorHeight(setpoint * 2);
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
