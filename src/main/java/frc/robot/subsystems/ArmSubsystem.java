// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.NeoMotorConstants;
import frc.robot.constants.ArmConstants.ArmAngles;
import frc.robot.constants.ArmConstants.IntakeDirection;
import frc.robot.utils.Utility;

public class ArmSubsystem extends SubsystemBase {

  double[] data = new double[10];
  int index = 0;
  double avg = 0;
  double currentData = 0;
  double lastData = 0;
  private final SparkMax pitchSpark = new SparkMax(ArmConstants.A_PITCH_ID, MotorType.kBrushless);
  private final RelativeEncoder pitchEncoder = pitchSpark.getEncoder();
  private final DutyCycleEncoder lampreyPWM = new DutyCycleEncoder(7, 360, 0);

  private final SparkMax openSpark = new SparkMax(ArmConstants.A_OPEN_ID, MotorType.kBrushless);

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
        .disableFollowerMode();

    basicConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)

            .maxMotion
        .allowedClosedLoopError(ArmConstants.A_TOLERANCE, slot0)
        .maxAcceleration(NeoMotorConstants.NEO_MAX_ACC)
        .maxVelocity(NeoMotorConstants.NEO_MAX_VEL)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal, slot0);

    pitchConfig
        .smartCurrentLimit(ArmConstants.PITCH_STALL_LIMIT, NeoMotorConstants.NEO_FREE_LIMIT)
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        
    .encoder
        .positionConversionFactor(ArmConstants.pConversionFactor);

    pitchConfig.closedLoop
    .minOutput(-0.6)
    .maxOutput(0.6)
        .pidf(
            ArmConstants.kP_PITCH,
            ArmConstants.kI_PITCH,
            ArmConstants.kD_PITCH,
            ArmConstants.kF_PITCH,
            slot0);

    // pitchConfig.absoluteEncoder
    // .positionConversionFactor(ArmConstants.pConversionFactor)
    // .zeroOffset(0); // change to actual value

    openConfig
        .smartCurrentLimit(ArmConstants.OPEN_STALL_LIMIT, NeoMotorConstants.NEO_FREE_LIMIT)
        .inverted(false)
        .idleMode(IdleMode.kCoast)

    .closedLoop
        .pidf(
            ArmConstants.kP_OPEN,
            ArmConstants.kI_OPEN,
            ArmConstants.kD_OPEN,
            ArmConstants.kF_OPEN,
            slot0);
  }

  public double getLampreyPose(){
    return lampreyPWM.get();
  }

  public double getEncoderPose() {
    return pitchEncoder.getPosition();
  }

  public void resetPitch(){
    pitchEncoder.setPosition(0);
  }

  public void stopPitch(){
    pitchSpark.stopMotor();
  }

  public void burnFlash() {
    pitchSpark.configure(basicConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    openSpark.configure(basicConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pitchSpark.configure(pitchConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    openSpark.configure(openConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public double getOpenCurrent(){
    return openSpark.getOutputCurrent();
  }

  public double getComparison(double tar){
    return (getEncoderPose() - avg + 38) + tar;
  }

  public void manualPitchMotor(double volts) {
    pidPitch.setReference(volts * 1.5, ControlType.kVoltage, slot0, 0.0, ArbFFUnits.kVoltage);
  }

  public Command intakeOuttake(double intake){
    return run(() -> pidOpen.setReference(intake, ControlType.kDutyCycle, slot0));
  }

  public Command intakeOuttake(IntakeDirection direction) { // ex 1 or 0
    return run(() -> pidOpen.setReference(direction.getPower(), ControlType.kDutyCycle, slot0))
      .finallyDo(() -> pitchSpark.stopMotor());
  }

  public Command setWithLamprey(double tar){
    return run(() -> pidPitch.setReference(getComparison(tar), ControlType.kPosition, slot0));
  }

  public Command setWithLamprey(ArmAngles angle){
    return run(() -> pidPitch.setReference(getComparison(angle.getAngle()), ControlType.kPosition, slot0));
  }

  public Command setPitch(double tar){
    return run(() -> pidPitch.setReference(tar, ControlType.kPosition, slot0));
  }

  public Command setPitch(ArmAngles angle){
    return run(() -> pidPitch.setReference(angle.getAngle(), ControlType.kPosition, slot0));
  }

  public Command zero(){
    return (setWithLamprey(ArmAngles.ZERO)
    .until(() -> isAtPoint(getComparison(0))))
    .andThen(() -> stopPitch())
    .andThen(() -> resetPitch());
  }

  public boolean isAtTarget(){
    return Utility.withinTolerance(getEncoderPose(), target, ArmConstants.A_TOLERANCE);
  }

  public boolean isAtPoint(double point){
    return Utility.withinTolerance(getEncoderPose(), point, ArmConstants.A_TOLERANCE);
  }

  public boolean isAtPoint(ArmAngles angle){
    return Utility.withinTolerance(getEncoderPose(), angle.getAngle(), ArmConstants.A_TOLERANCE);
  }

  public boolean isSafe(){
    return avg > 20;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm/Arm Encoder: ", getEncoderPose());
    SmartDashboard.putNumber("Arm/Lamprey Analog Reading: ", getLampreyPose());
    SmartDashboard.putNumber("Arm/Lamprey Frequency: ", lampreyPWM.getFrequency());
    SmartDashboard.putNumber("Arm/Arm Target: ", target);
    SmartDashboard.putBoolean("Arm/At Target", isAtTarget());

    currentData = getLampreyPose();

  if(Math.abs(lastData - currentData) < 45 ){
    data[index] = currentData;
    index++;
    index = index % 10;
   
  }
  lastData = currentData;
  avg = 0;
    for(double num : data) avg += num;
    avg = avg / 10.0;

    if(avg > 190) avg -= 360;
    if(avg < -180) avg += 360;
  
    SmartDashboard.putNumber("Arm/Lamprey Average: ", avg);
    SmartDashboard.putNumber("Arm/Lamprey Comparison: ", getComparison(0));
    SmartDashboard.putBoolean("Arm/Is Safe For CGI 0: ", isSafe());
  }
}