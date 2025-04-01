package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.CoralGroundIntakeConstants;
import frc.robot.constants.CoralGroundIntakeConstants.CoralAngles;
import frc.robot.constants.NeoMotorConstants;
import frc.robot.utils.Utility;

public class CoralGroundIntake extends SubsystemBase {

  // Front and Back intake motors
  private final SparkMax frontIntakeMotor = new SparkMax(CoralGroundIntakeConstants.FRONT_PICKUP_MOTOR_ID,
      MotorType.kBrushless);
  private final SparkMax backIntakeMotor = new SparkMax(CoralGroundIntakeConstants.BACK_PICKUP_MOTOR_ID,
      MotorType.kBrushless);
  // Single rotate motor for the arm
  private final SparkMax rotateMotor = new SparkMax(CoralGroundIntakeConstants.ROTATE_MASTER_ID, MotorType.kBrushless);

  // Motors Configs
  private final SparkMaxConfig frontIntakeConfig = new SparkMaxConfig();
  private final SparkMaxConfig backIntakeConfig = new SparkMaxConfig();
  private final SparkMaxConfig rotateConfig = new SparkMaxConfig();
  private final ClosedLoopSlot slot0 = CoralGroundIntakeConstants.INTAKE_SLOT;

  // PID and Encoder Stuff
  private final SparkClosedLoopController rotationPID = rotateMotor.getClosedLoopController();
  private final RelativeEncoder rotationEncoder = rotateMotor.getEncoder();

  // Sensor
  private final DigitalInput beamBreakSensor = new DigitalInput(CoralGroundIntakeConstants.BEAM_BREAK_SENSOR_ID);

  private final double tolerance = CoralGroundIntakeConstants.ROTATION_TOLERANCE;

  public CoralGroundIntake() {
    // Configure Front Intake Motor
    frontIntakeConfig
        .smartCurrentLimit(CoralGroundIntakeConstants.PICKUP_MOTOR_STALL_LIMIT, NeoMotorConstants.NEO_FREE_LIMIT)
        .voltageCompensation(NeoMotorConstants.NEO_NOMINAL_VOLTAGE)
        .idleMode(IdleMode.kCoast)
        .inverted(true);

    // Configure Back Intake Motor
    backIntakeConfig
        .smartCurrentLimit(CoralGroundIntakeConstants.PICKUP_MOTOR_STALL_LIMIT, NeoMotorConstants.NEO_FREE_LIMIT)
        .voltageCompensation(NeoMotorConstants.NEO_NOMINAL_VOLTAGE)
        .idleMode(IdleMode.kCoast)
        .inverted(false);

    // Configure Rotate Motor with PID settings
    rotateConfig
        .smartCurrentLimit(CoralGroundIntakeConstants.ROTATION_MOTOR_STALL_LIMIT, NeoMotorConstants.NEO_FREE_LIMIT)
        .voltageCompensation(NeoMotorConstants.NEO_NOMINAL_VOLTAGE)
        .idleMode(IdleMode.kBrake)
        .inverted(false);

    rotateConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .minOutput(-0.9)
        .maxOutput(0.9)
        .pid(CoralGroundIntakeConstants.ROTATION_kP,
            CoralGroundIntakeConstants.ROTATION_kI,
            CoralGroundIntakeConstants.ROTATION_kD,
            slot0);
  }

  // Call this method once during initialization to store settings to flash
  public void burnFlash() {
    frontIntakeMotor.configure(frontIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    backIntakeMotor.configure(backIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rotateMotor.configure(rotateConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void intakeFront(){
    frontIntakeMotor.set(0.7);
  }

  public void intakeBack(){
    backIntakeMotor.set(0.82);
  }

  public void outakeFront(){
    frontIntakeMotor.set(-0.2);
  }

  public void outakeBack(){
    backIntakeMotor.set(-0.5);
  }

  public void intakeBoth(){
     intakeFront();
     intakeBack();
  }

  public void outakeBoth(){
    outakeFront();
    outakeBack();
  }

  public Command outtake(){
    outakeBoth();
    if(beamBreakSensor.get()){
      return new WaitCommand(2)
      .andThen(() -> stopIntake());
    }
    else return run(() -> outakeBoth());
  }

  public void stopIntake(){
    frontIntakeMotor.stopMotor();
    backIntakeMotor.stopMotor();
  }

  // Main Intake Sequence
  public Command intakeSequence() {
    return new SequentialCommandGroup(intakeWithBeamBreak(),
    setIntakePosition(CoralAngles.CORAL).until(() -> isAtPoint(CoralAngles.CORAL)));
  }

  public void stopRotate(){
    rotateMotor.stopMotor();
  }

  // Set Intake Arm Position using PID
  public Command setIntakePosition(double position) {
    return run(() -> rotationPID.setReference(position, ControlType.kPosition, slot0));
  }

  public Command setIntakePosition(CoralAngles position) {
    return run(() -> rotationPID.setReference(position.getAngle(), ControlType.kPosition, slot0));
  }

  public Command intakeWithBeamBreak() {
    return (setIntakePosition(CoralAngles.FLOOR).until(() -> isAtPoint(CoralAngles.FLOOR) || rotateMotor.getOutputCurrent() > CoralGroundIntakeConstants.ROTATION_MOTOR_STALL_LIMIT))
      .andThen(run(() -> intakeBoth())
        .onlyWhile(() -> beamBreakSensor.get()))
      .finallyDo(() -> stopIntake());
  }

  public boolean isAtPoint(double point){
    return Utility.withinTolerance(getEncoderPose(), point + 0.11, 0.08);
  }

  public boolean isAtPoint(CoralAngles point){
    return Utility.withinTolerance(getEncoderPose(), point.getAngle() + 0.11, 0.08);
  }

  public double getEncoderPose(){
    return rotationEncoder.getPosition();
  }

  public Command runCurrentZeroing(){
    return run(() -> rotateMotor.setVoltage(2))
    .until(() -> rotateMotor.getOutputCurrent() > CoralGroundIntakeConstants.ROTATION_MOTOR_STALL_LIMIT)
    .andThen(() -> rotateMotor.stopMotor())
    .finallyDo(() -> rotationEncoder.setPosition(0));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake/Front Motor Current", frontIntakeMotor.getOutputCurrent());
    SmartDashboard.putNumber("Intake/Back Motor Current", backIntakeMotor.getOutputCurrent());
    SmartDashboard.putNumber("Intake/Rotate Motor Current", rotateMotor.getOutputCurrent());
    SmartDashboard.putNumber("Intake/Bus Voltage: ", rotateMotor.getBusVoltage());
    SmartDashboard.putNumber("Intake/Arm Position", rotationEncoder.getPosition());
    SmartDashboard.putBoolean("Intake/Beam Break Sensor", beamBreakSensor.get());
  }
}
