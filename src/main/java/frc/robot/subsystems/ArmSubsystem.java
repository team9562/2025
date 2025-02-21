// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.NeoMotorConstants;

public class ArmSubsystem extends SubsystemBase {

  private final SparkMax armSpark = new SparkMax(ArmConstants.ARM_ID, MotorType.kBrushless);
  private final RelativeEncoder armEncoder = armSpark.getEncoder();
  private final SparkMaxConfig armConfig = new SparkMaxConfig();
  private final SparkClosedLoopController armPid = armSpark.getClosedLoopController();

  private final double kP = ArmConstants.kP;
  private final double kI = ArmConstants.kI;
  private final double kD = ArmConstants.kD;
  private final ClosedLoopSlot ARM_SLOT = ArmConstants.ARM_SLOT;

  public ArmSubsystem() {

    armConfig
      .smartCurrentLimit(ArmConstants.ARM_STALL_LIMIT, NeoMotorConstants.NEO_FREE_LIMIT, NeoMotorConstants.NEO_MAX_RPM)
      .voltageCompensation(NeoMotorConstants.NEO_NOMINAL_VOLTAGE);

    armConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(kP, kI, kD, ARM_SLOT);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
