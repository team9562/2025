// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.utils.Utility;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ScoreCoral extends Command {

  ElevatorSubsystem m_elevator;
  ArmSubsystem m_arm;
  String target;
  double setpoint;
  double angle;
  /** Creates a new score. */
  public ScoreCoral(ElevatorSubsystem sub1, ArmSubsystem sub2, String targetHeight) {
    m_elevator = sub1;
    m_arm = sub2;
    target = targetHeight;
    addRequirements(m_elevator, m_arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(target.equalsIgnoreCase("l2")){
      setpoint = ElevatorConstants.L2;
    }

    else if(target.equalsIgnoreCase("l3")){
      setpoint = ElevatorConstants.L3;
    }

    else if(target.equalsIgnoreCase("l4")){
      setpoint = ElevatorConstants.L4;
    }

    else if(target.equalsIgnoreCase("b")){
      setpoint = ElevatorConstants.B;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevator.setElevatorHeight(setpoint);

    if(Utility.betweenRange(m_elevator.getEncoderPose(), setpoint - 5, setpoint + 5)){
      m_arm.turnPitchMotor(angle);
    }

    /*if(arm's encoder = the angle && elevator's encoder = the setpoint)
    {outtake}
    */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
