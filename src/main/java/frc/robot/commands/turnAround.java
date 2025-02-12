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
public class turnAround extends Command {

  private final CommandSwerveDrivetrain ta_drivetrain;
  private final SwerveRequest.FieldCentric ta_drive;
  private Pigeon2 imu;
  private double target;
  private double currentYaw;
  private double ta_AngularRate;

  public turnAround(CommandSwerveDrivetrain subsystem, FieldCentric request, double angularRate) {

    this.ta_drivetrain = subsystem;
    this.ta_drive = request;
    this.ta_AngularRate = angularRate;
    imu = ta_drivetrain.getPigeon2();

    addRequirements(ta_drivetrain);
  }

  private double getPigeonYaw(){
    return Math.abs(Math.round(imu.getAngle() % 360));
  }

  public void outputYawTargetError(){
    System.out.println("Yaw: " + currentYaw);
    System.out.println("Target: " + target);
    System.out.println("Error: " + (currentYaw < target ? currentYaw + target : currentYaw - target));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    //if it should subtract 180 or add 180 to the value
    //imu.setYaw(0);
    currentYaw = getPigeonYaw();
    target = currentYaw < 180 ? currentYaw + 180 : currentYaw - 180;
    target = Math.round(target/10)*10;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //sets the wheel direction
    this.ta_drivetrain.setControl(ta_drive.withRotationalRate(1 * ta_AngularRate));

    this.currentYaw = getPigeonYaw();

    //debug info
    System.out.println("command is executing");
    outputYawTargetError();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("turnAround complete");
    System.out.println("FINAL VALUES: ");
    outputYawTargetError();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //stops when the yaw is at the same rotation as the target
    return Math.round(currentYaw/10)*10 == target - 30;
  }
}
