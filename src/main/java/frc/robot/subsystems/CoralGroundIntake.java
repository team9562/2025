package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CoralGroundIntakeConstants;
import frc.robot.constants.NeoMotorConstants;
import frc.robot.constants.CoralGroundIntakeConstants.CoralAngles;
import frc.robot.utils.Utility;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

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
  private final AbsoluteEncoder rotationEncoder = rotateMotor.getAbsoluteEncoder();

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
        .minOutput(-0.75)
        .maxOutput(0.75)
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
    frontIntakeMotor.set(0.2);
  }

  public void intakeBack(){
    backIntakeMotor.set(0.5);
  }

  public void intakeBoth(){
     intakeFront();
     intakeBack(); // this might be an error but prolly not
  }

  public void stopIntake(){
    frontIntakeMotor.stopMotor();
    backIntakeMotor.stopMotor();
  }

  public Command IntakeWithBeamBreak() {
    return run(() -> setIntakePosition(CoralAngles.FLOOR).until(() -> isAtPoint(CoralAngles.FLOOR)))
        .andThen(run(() -> intakeBoth())
          .until(() -> beamBreakSensor.get()))
        .finallyDo(() -> stopIntake());
  }

  // Main Intake Sequence
  public Command intakeSequence() {
    return new SequentialCommandGroup(IntakeWithBeamBreak(),
    setIntakePosition(CoralAngles.CORAL)
    );
  }

  // Set Intake Arm Position using PID
  public Command setIntakePosition(double position) {
    return run(() -> rotationPID.setReference(position, ControlType.kPosition, slot0));
  }

  public Command setIntakePosition(CoralAngles position) {
    return run(() -> rotationPID.setReference(position.getAngle(), ControlType.kPosition, slot0));
  }

  public boolean isAtPoint(double point){
    return Utility.withinTolerance(getEncoderPose(), point, tolerance);
  }

  public boolean isAtPoint(CoralAngles point){
    return Utility.withinTolerance(getEncoderPose(), point.getAngle(), tolerance);
  }

  public double getEncoderPose(){
    return rotationEncoder.getPosition();
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
