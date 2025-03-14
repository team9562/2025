// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.utils.Utility;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetAngleToPOI extends Command {
  ArmSubsystem m_arm;
  String poi;
  double targetAngle;

  /** Creates a new SetAngleToPOI. */
  public SetAngleToPOI(ArmSubsystem sub1, String POI) {
    this.m_arm = sub1;
    this.poi = POI;
    addRequirements(m_arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (poi.toLowerCase()) {
      default:
        targetAngle = m_arm.getEncoderPose();
        break;

      case "l2":
        targetAngle = -103;
        break;

      case "l3":
        targetAngle = -103;
        break;

      case "l4":
        targetAngle = -129.94;
        break;

      case "b":
        targetAngle = 0;
        break;

      case "coral":
        targetAngle = 38;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.turnPitchMotor(targetAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Utility.withinTolerance(m_arm.getEncoderPose(), targetAngle, 1);
  }
}
