// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeOutake extends Command {
  /** Creates a new IntakeOutake. */
  ArmSubsystem m_ArmSubsystem;
  String direction;
  double speed;
  Timer elapsed = new Timer();

  public IntakeOutake(ArmSubsystem arm, String inOutStop) {
    this.m_ArmSubsystem = arm;
    this.direction = inOutStop;
    addRequirements(m_ArmSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elapsed.reset();
    elapsed.start();

    switch (direction.toLowerCase()) {
      default:
        speed = 0;
        break;

      case "in":
        speed = 1;
        break;

      case "out":
        speed = -1;
        break;

      case "stop":
        speed = 0;
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ArmSubsystem.turnOpenMotor(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ArmSubsystem.stopOpen();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elapsed.hasElapsed(1); // idk guess a better value or use lasercan data to stop
  }
}
