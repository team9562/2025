// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.ArmCommands.SetAngleToPOI;
import frc.robot.commands.ElevatorCommands.SetHeightToPOI;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetHeightAngleToPOI extends ParallelCommandGroup {
  ElevatorSubsystem m_ElevatorSubsystem;
  ArmSubsystem m_ArmSubsystem;
  String poi;
  /** Creates a new SetHeightAngleToPOI. */
  public SetHeightAngleToPOI(ElevatorSubsystem sub1, ArmSubsystem sub2, String POI) {
    this.m_ElevatorSubsystem = sub1;
    this.m_ArmSubsystem = sub2;
    this.poi = POI;
    addCommands(new SetHeightToPOI(m_ElevatorSubsystem, poi), new SetAngleToPOI(m_ArmSubsystem, poi));
  }
}
