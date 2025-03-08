// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.IntakeConstants;
import frc.robot.constants.NeoMotorConstants;
import frc.robot.utils.Utility;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSubsystem extends SubsystemBase {

  // Intake Motors
  private final SparkMax intakeLeft  = new SparkMax(IntakeConstants.INTAKE_LEFT_ID,  MotorType.kBrushless);
  private final SparkMax intakeRight = new SparkMax(IntakeConstants.INTAKE_RIGHT_ID, MotorType.kBrushless);

  private final SparkClosedLoopController leftPID  = intakeLeft.getClosedLoopController();
  private final SparkClosedLoopController rightPID = intakeRight.getClosedLoopController();

  // Encoders
  private final RelativeEncoder leftEncoder  = intakeLeft.getEncoder();
  private final RelativeEncoder rightEncoder = intakeRight.getEncoder();

  // Configuration objects for each SparkMax
  private final SparkMaxConfig leftConfig  = new SparkMaxConfig();
  private final SparkMaxConfig rightConfig = new SparkMaxConfig();

  // Tolerance and target for closed-loop control
  private final double tolerance = IntakeConstants.INTAKE_TOLERANCE;
  private double target    = 0.0;

  public static final ClosedLoopSlot E_SLOT = ClosedLoopSlot.kSlot0;

  public IntakeSubsystem() {

    // -----------------------------
    // lEFT MOTOR CONFIG
    // -----------------------------
    leftConfig
      // Current limiting
      .smartCurrentLimit(
          IntakeConstants.I_STALL_LIMIT, 
          NeoMotorConstants.NEO_FREE_LIMIT, 
          NeoMotorConstants.NEO_MAX_RPM
      )
      .voltageCompensation(NeoMotorConstants.NEO_NOMINAL_VOLTAGE)
      .idleMode(IdleMode.kBrake)
      .inverted(false);

    // Closed-loop config
    leftConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      // PIDF gains
      .pidf(
          IntakeConstants.kP, 
          IntakeConstants.kI, 
          IntakeConstants.kD, 
          IntakeConstants.kFF, 
          IntakeConstants.INTAKE_SLOT
      )

      .maxMotion
        .allowedClosedLoopError(tolerance, IntakeConstants.INTAKE_SLOT)
        .maxAcceleration(NeoMotorConstants.NEO_MAX_ACC, IntakeConstants.INTAKE_SLOT)
        .maxVelocity(NeoMotorConstants.NEO_MAX_VEL, IntakeConstants.INTAKE_SLOT)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal, IntakeConstants.INTAKE_SLOT);

    // Encoder conversion factors
    leftConfig.encoder
      .positionConversionFactor(IntakeConstants.kPositionConversionFactor)
      .velocityConversionFactor(IntakeConstants.kVelocityConversionFactor);

    // -----------------------------
    // RIGHT MOTOR CONFIG
    // -----------------------------
    rightConfig
      .smartCurrentLimit(
          IntakeConstants.I_STALL_LIMIT, 
          NeoMotorConstants.NEO_FREE_LIMIT, 
          NeoMotorConstants.NEO_MAX_RPM
      )
      .voltageCompensation(NeoMotorConstants.NEO_NOMINAL_VOLTAGE)
      .idleMode(IdleMode.kBrake)
      // Possibly invert the right motor if needed
      .inverted(true);

    rightConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pidf(
          IntakeConstants.kP, 
          IntakeConstants.kI, 
          IntakeConstants.kD, 
          IntakeConstants.kFF, 
          IntakeConstants.INTAKE_SLOT
      )
      .maxMotion
        .allowedClosedLoopError(tolerance, IntakeConstants.INTAKE_SLOT)
        .maxAcceleration(NeoMotorConstants.NEO_MAX_ACC, IntakeConstants.INTAKE_SLOT)
        .maxVelocity(NeoMotorConstants.NEO_MAX_VEL, IntakeConstants.INTAKE_SLOT)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal, IntakeConstants.INTAKE_SLOT);

    rightConfig.encoder
      .positionConversionFactor(IntakeConstants.kPositionConversionFactor)
      .velocityConversionFactor(IntakeConstants.kVelocityConversionFactor);

    // If you want one motor to follow the other, uncomment this: (Ask LUCAS)
    // rightConfig.follow(IntakeConstants.INTAKE_LEFT_ID, true);

    // Actually apply (burn) the configs to the SparkMax
    burnFlash();
  }

  // Pushes all the config changes to the Spark MAX controllers and saves them.
  public void burnFlash() {
    intakeLeft.configure(leftConfig,  ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeRight.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }


  public Command runIntake(double speed) {
    return this.run(() -> {
      intakeLeft.set(speed);
      intakeRight.set(speed);
    });
  }

  public void stopIntake() {
    intakeLeft.stopMotor();
    intakeRight.stopMotor();
  }

  public void setIntakePosition(double position) {
    this.target = position;
    leftPID.setReference(position, ControlType.kMAXMotionPositionControl, IntakeConstants.INTAKE_SLOT);
    rightPID.setReference(position, ControlType.kMAXMotionPositionControl, IntakeConstants.INTAKE_SLOT);
  }

  // Pos of the encoders
  public double getPosition() {
    return leftEncoder.getPosition();
  }

  // Check if we are close enough to the target position.
  public boolean isAtTarget() {
    return Utility.withinTolerance(getPosition(), target, tolerance);
  }

  // Zero both encoders.
  public void resetEncoders() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  // For testing and debugging
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Left Current", intakeLeft.getOutputCurrent());
    SmartDashboard.putNumber("Intake Right Current", intakeRight.getOutputCurrent());
    SmartDashboard.putNumber("Intake Position", getPosition());
    SmartDashboard.putBoolean("Intake At Target", isAtTarget());
  }
}