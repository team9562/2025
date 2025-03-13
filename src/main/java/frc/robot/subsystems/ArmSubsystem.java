// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.NeoMotorConstants;
import frc.robot.utils.Utility;

public class ArmSubsystem extends SubsystemBase {

  private final SparkMax pitchSpark = new SparkMax(ArmConstants.A_PITCH_ID, MotorType.kBrushless);
  private final RelativeEncoder pitchEncoder = pitchSpark.getEncoder();
  private final AbsoluteEncoder absoluteZeroPitch = pitchSpark.getAbsoluteEncoder();

  private final SparkMax openSpark = new SparkMax(ArmConstants.A_OPEN_ID, MotorType.kBrushless);
  private final RelativeEncoder openEncoder = openSpark.getEncoder();

  private final SparkMaxConfig basicConfig = new SparkMaxConfig();
  private final SparkMaxConfig pitchConfig = new SparkMaxConfig();
  private final SparkMaxConfig openConfig = new SparkMaxConfig();
  private final SparkClosedLoopController pidPitch = pitchSpark.getClosedLoopController();
  private final SparkClosedLoopController pidOpen = openSpark.getClosedLoopController();
  private final ClosedLoopSlot slot0 = ArmConstants.ARM_SLOT;

  private double target;

  public ArmSubsystem() {
    basicConfig
        .voltageCompensation(NeoMotorConstants.NEO_NOMINAL_VOLTAGE)
        .disableFollowerMode()
        .idleMode(IdleMode.kBrake);

    basicConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)

            .maxMotion
        .allowedClosedLoopError(ArmConstants.A_TOLERANCE, slot0)
        .maxAcceleration(NeoMotorConstants.NEO_MAX_ACC)
        .maxVelocity(NeoMotorConstants.NEO_MAX_VEL)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal, slot0);

    pitchConfig
        .smartCurrentLimit(ArmConstants.PITCH_STALL_LIMIT, NeoMotorConstants.NEO_FREE_LIMIT)
        .inverted(false).encoder
        .positionConversionFactor(ArmConstants.pConversionFactor);

    pitchConfig.closedLoop
        .pidf(
            ArmConstants.kP_PITCH,
            ArmConstants.kI_PITCH,
            ArmConstants.kD_PITCH,
            ArmConstants.kF_PITCH,
            slot0);

    pitchConfig.absoluteEncoder
        .positionConversionFactor(ArmConstants.pConversionFactor)
        .zeroOffset(0); // change to actual value

    openConfig
        .smartCurrentLimit(ArmConstants.OPEN_STALL_LIMIT, NeoMotorConstants.NEO_FREE_LIMIT)
        .inverted(false);

    openConfig.closedLoop
        .pidf(
            ArmConstants.kP_OPEN,
            ArmConstants.kI_OPEN,
            ArmConstants.kD_OPEN,
            ArmConstants.kF_OPEN,
            slot0);
  }

  public void resetPitch() {
    pitchEncoder.setPosition(0);
  }

  public double getEncoderPose() {
    return pitchEncoder.getPosition() % 360;
  }

  public void burnFlash() {
    pitchSpark.configure(basicConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    openSpark.configure(basicConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pitchSpark.configure(pitchConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    openSpark.configure(openConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void stopPitch() {
    pitchSpark.stopMotor();
  }

  public void stopOpen() {
    openSpark.stopMotor();
  }

  public static double getShortestRotation(double currentAngle, double targetAngle) {
    double difference = (targetAngle - currentAngle) % 360;

    if (difference > 180) {
      difference -= 360;
    } else if (difference < -180) {
      difference += 360;
    }

    return difference;
  }

  public void turnPitchMotor(double degrees) {
    pidPitch.setReference(getShortestRotation(getEncoderPose(), degrees), ControlType.kMAXMotionPositionControl, slot0);
  }

  public Command turnOpenMotor(double intake) { // ex 1 or 0
    return this
        .run(() -> pidOpen.setReference(intake, ControlType.kDutyCycle, slot0));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Encoder: ", getEncoderPose());
    if (Utility.withinTolerance(getEncoderPose(), absoluteZeroPitch.getPosition(), target)) {
      resetPitch();
    }
  }
}
