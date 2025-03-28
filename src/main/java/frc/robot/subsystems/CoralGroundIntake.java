package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CoralGroundIntakeConstants;
import frc.robot.constants.NeoMotorConstants;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class CoralGroundIntake extends SubsystemBase {

  // Front and Back intake motors
  private final SparkMax frontIntakeMotor = new SparkMax(CoralGroundIntakeConstants.FRONT_PICKUP_MOTOR_ID, MotorType.kBrushless);
  private final SparkMax backIntakeMotor  = new SparkMax(CoralGroundIntakeConstants.BACK_PICKUP_MOTOR_ID, MotorType.kBrushless);
  // Single rotate motor for the arm
  private final SparkMax rotateMotor = new SparkMax(CoralGroundIntakeConstants.ROTATE_MASTER_ID, MotorType.kBrushless);

  // Motors Configs
  private final SparkMaxConfig frontIntakeConfig = new SparkMaxConfig();
  private final SparkMaxConfig backIntakeConfig = new SparkMaxConfig();
  private final SparkMaxConfig rotateConfig = new SparkMaxConfig();

  // PID and Encoder Stuff
  private final SparkClosedLoopController rotationPID = rotateMotor.getClosedLoopController();
  private final AbsoluteEncoder rotationEncoder = rotateMotor.getAbsoluteEncoder();

  // Sensor 
  private final DigitalInput beamBreakSensor = new DigitalInput(CoralGroundIntakeConstants.BEAM_BREAK_SENSOR_ID);
  private final ClosedLoopSlot slot0 = CoralGroundIntakeConstants.INTAKE_SLOT;

  // Other Constants 
  private final double targetRotation = 5.0; // in degrees btw
  private final double rotationTolerance = CoralGroundIntakeConstants.ROTATION_TOLERANCE;

  public CoralGroundIntake() {
    // Configure Front Intake Motor
    frontIntakeConfig
        .smartCurrentLimit(CoralGroundIntakeConstants.PICKUP_MOTOR_STALL_LIMIT, NeoMotorConstants.NEO_FREE_LIMIT)
        .voltageCompensation(NeoMotorConstants.NEO_NOMINAL_VOLTAGE)
        .idleMode(IdleMode.kBrake)
        .inverted(true);

    // Configure Back Intake Motor
    backIntakeConfig
        .smartCurrentLimit(CoralGroundIntakeConstants.PICKUP_MOTOR_STALL_LIMIT, NeoMotorConstants.NEO_FREE_LIMIT)
        .voltageCompensation(NeoMotorConstants.NEO_NOMINAL_VOLTAGE)
        .idleMode(IdleMode.kBrake)
        .inverted(false);

    // Configure Rotate Motor with PID settings
    rotateConfig
        .smartCurrentLimit(CoralGroundIntakeConstants.ROTATION_MOTOR_STALL_LIMIT, NeoMotorConstants.NEO_FREE_LIMIT)
        .voltageCompensation(NeoMotorConstants.NEO_NOMINAL_VOLTAGE)
        .idleMode(IdleMode.kCoast)
        .inverted(false);

    rotateConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pidf(CoralGroundIntakeConstants.ROTATION_kP,
              CoralGroundIntakeConstants.ROTATION_kI,
              CoralGroundIntakeConstants.ROTATION_kD,
              CoralGroundIntakeConstants.ROTATION_kFF,
              slot0);
  }

  // Call this method once during initialization to store settings to flash
  public void burnFlash() {
    frontIntakeMotor.configure(frontIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    backIntakeMotor.configure(backIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rotateMotor.configure(rotateConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // Main Intake Sequence 
  public Command intakeSequence() {
    return (run(() -> frontIntakeMotor.set(1.0)).withTimeout(3))
        .andThen(setIntakePosition(targetRotation))
        .andThen(run(() -> backIntakeMotor.set(1.0)))
        .until(() -> beamBreakSensor.get())
        .finallyDo(() -> {
          frontIntakeMotor.stopMotor();
          backIntakeMotor.stopMotor();
          rotateMotor.stopMotor();
        });
  }

    public Command frontIntakeAndPositionCommand() {
        return (run(() -> frontIntakeMotor.set(0.19)).withTimeout(3))
            .andThen(setIntakePosition(targetRotation));
    }
 
    public Command backIntakeUntilBeamBreakCommand() {
        return run(() -> backIntakeMotor.set(0.5))
            .until(() -> beamBreakSensor.get())
            .finallyDo(() -> {
            frontIntakeMotor.stopMotor();
            backIntakeMotor.stopMotor();
            rotateMotor.stopMotor();
            });
    }


  // Set Intake Arm Position using PID 
  public Command setIntakePosition(double position) {
    return run(() -> rotationPID.setReference(position, ControlType.kPosition, CoralGroundIntakeConstants.INTAKE_SLOT ));
  }

  // Manual Control Stuff 

  public void manualFrontIntake(double speed) {
    frontIntakeMotor.set(speed);
  }

  public void manualBackIntake(double speed) {
    backIntakeMotor.set(speed);
  }

  public void manualRotate(double speed) {
    rotateMotor.set(speed);
  }

  public Command logBeamBreakStatus() {
    return new InstantCommand(() -> {
      System.out.println("Beam Break Sensor: " + (beamBreakSensor.get() ? "Unbroken" : "Broken"));
    }, this);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake/Front Motor Current", frontIntakeMotor.getOutputCurrent());
    SmartDashboard.putNumber("Intake/Back Motor Current", backIntakeMotor.getOutputCurrent());
    SmartDashboard.putNumber("Intake/Rotate Motor Current", rotateMotor.getOutputCurrent());
    SmartDashboard.putNumber("Intake/Arm Position", rotationEncoder.getPosition());
    SmartDashboard.putBoolean("Intake/Beam Break Sensor", beamBreakSensor.get());
    System.out.println(rotationEncoder.getPosition());
  }
}
