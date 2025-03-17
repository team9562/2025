// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.followGuzPath;

import frc.robot.commands.IntakeCommands.GroundIntakeCommand;
import frc.robot.commands.LEDCommands.SetLedStateCommand;
import frc.robot.commands.SwerveCommands.GoToBestTargetCommand;
import frc.robot.commands.SwerveCommands.TurnAroundCommand;
import frc.robot.commands.SwerveCommands.TurnToBestTargetCommand;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ArmConstants.ArmAngles;
import frc.robot.constants.ArmConstants.IntakeDirection;
import frc.robot.constants.ElevatorConstants.ElevatorHeights;
import frc.robot.constants.GroundIntakeConstants.GroundIntakeSetpoint;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem;
import frc.robot.subsystems.LaserCanSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.LedSubsystem.RobotState;
import frc.robot.utils.Utility;

public class RobotContainer {

    // speed is divided by 3 to accommodate for small testing spaces
    public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                        // speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second -
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.3)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    // private final Telemetry logger = new Telemetry(MaxSpeed);

    static CommandXboxController XController = new CommandXboxController(0);

    public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final static ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
    public final static ArmSubsystem m_armSubsystem = new ArmSubsystem();
    public final static GroundIntakeSubsystem m_groundIntakeSubsystem = new GroundIntakeSubsystem();
    public final static VisionSubsystem m_visionSubsystem = new VisionSubsystem();
    public final static LaserCanSubsystem m_laserCan = new LaserCanSubsystem();
    public final LedSubsystem ledSubsystem = new LedSubsystem();

    private final SendableChooser<Command> autoChooser;

    private final Command intakeCoralAlgae() {

        if (m_laserCan.processMeasurement()) {
            return m_armSubsystem.intakeOuttake(IntakeDirection.IN)
                    .until(() -> m_armSubsystem.getOpenCurrent() > ArmConstants.OPEN_STALL_LIMIT)
                    .andThen(m_armSubsystem.intakeOuttake(0.4)); // find a good percent to hold the algae in
        }

        else if (!m_laserCan.processMeasurement()) {
            return m_armSubsystem.intakeOuttake(IntakeDirection.IN)
                    .until(() -> m_armSubsystem.getOpenCurrent() > ArmConstants.OPEN_STALL_LIMIT)
                    .finallyDo(() -> m_armSubsystem.intakeOuttake(IntakeDirection.STOP));
        }

        return m_armSubsystem.intakeOuttake(IntakeDirection.STOP);
    }

    private final Command homeElevatorArm() {
        return new ParallelCommandGroup(
                m_elevatorSubsystem.setElevatorHeight(ElevatorHeights.ZERO).until(m_elevatorSubsystem::isAtTarget).withTimeout(1),
                m_armSubsystem.turnPitchMotor(ArmAngles.ZERO).until(m_armSubsystem::isAtTarget).withTimeout(0.7))
                .andThen(m_armSubsystem.turnPitchMotor(ArmAngles.CORAL));
    }

    private final Command setHeightAngleToPOI(ArmAngles angle, ElevatorHeights height) {
        return m_armSubsystem.turnPitchMotor(ArmAngles.ZERO).until(m_armSubsystem::isAtTarget).withTimeout(0.7)
                .andThen(m_elevatorSubsystem.setElevatorHeight(height.getHeight())
                        .alongWith(m_armSubsystem.turnPitchMotor(angle.getAngle())));
    }

    private final Command autoScore = new SequentialCommandGroup(
            m_armSubsystem.turnPitchMotor(ArmAngles.ZERO).andThen(setHeightAngleToPOI(ArmAngles.L4, ElevatorHeights.L4))
                    .andThen(homeElevatorArm()));

    private final Command groundSafe(){
        return m_groundIntakeSubsystem.setIntakePosition(GroundIntakeSetpoint.IN)
        .onlyWhile(() -> m_elevatorSubsystem.getEncoderPose() > ElevatorHeights.OBIWAN.getHeight() && 
        Utility.betweenRange(m_armSubsystem.getEncoderPose(), -2, 2))
        .finallyDo(() -> m_groundIntakeSubsystem.stopRotation());
    }

    private final Command elevatorUpThenIntake(GroundIntakeSetpoint setpoint){
        return new SequentialCommandGroup(m_elevatorSubsystem.setElevatorHeight(ElevatorHeights.OBIWAN)
        .until(()-> m_elevatorSubsystem.isAtTarget()), m_groundIntakeSubsystem.setIntakePosition(setpoint));
    }

    private final Command turnToBestTargetCommand = new TurnToBestTargetCommand(drivetrain, m_visionSubsystem, drive,
            0);

    private final Command goToBestFunnel = new GoToBestTargetCommand(drivetrain, m_visionSubsystem, drive, true);
    private final Command goToBestNotFunnel = new GoToBestTargetCommand(drivetrain, m_visionSubsystem, drive, false);

    public RobotContainer() {

        registerCommands();
        autoChooser = AutoBuilder.buildAutoChooser("None");
        SmartDashboard.putData("Auto chooser", autoChooser);

        burnAllFlash();

        configureBindings();
    }

    private void registerCommands() {
        NamedCommands.registerCommand("turnToBestTarget", turnToBestTargetCommand);
        NamedCommands.registerCommand("goToBestFunnel", goToBestFunnel);
        NamedCommands.registerCommand("goToBestNotFunnel", goToBestNotFunnel);
        NamedCommands.registerCommand("ScoreCoral", autoScore);
        NamedCommands.registerCommand("Intake", m_armSubsystem.intakeOuttake(IntakeDirection.IN));
    }

    private void burnAllFlash() {
        m_elevatorSubsystem.burnFlash();
        m_armSubsystem.burnFlash();
        m_groundIntakeSubsystem.burnFlash();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(XController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(XController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(XController.getRightX() * 0)));

        m_armSubsystem.setDefaultCommand(m_armSubsystem.run(() -> m_armSubsystem.manualPitchMotor(XController.getRightY())));
        //m_visionSubsystem.setDefaultCommand(goToBestTargetCommand);
        ledSubsystem.setDefaultCommand(new SetLedStateCommand(ledSubsystem, RobotState.RAINBOW));
        // XController.b().onTrue(new SetLedStateCommand(ledSubsystem, RobotState.RAINBOW));

        // Change Around Please
        // XController.povUp().onChange(m_elevatorSubsystem.setElevatorHeight("l2"));
        XController.povUp().onChange(setHeightAngleToPOI(ArmAngles.B, ElevatorHeights.B)); // 67.17 in
        XController.povLeft().onChange(setHeightAngleToPOI(ArmAngles.L3, ElevatorHeights.L3)); // 43.86 in
        XController.povRight().onChange(setHeightAngleToPOI(ArmAngles.L2, ElevatorHeights.L2)); // 26.85 in
        XController.povDown().onChange(setHeightAngleToPOI(ArmAngles.L4, ElevatorHeights.L4)); // 77.1 in
        
        XController.y().onTrue(goToBestFunnel);
        XController.b().onTrue(goToBestNotFunnel); // no exit command rn -> fix later
        
        XController.rightStick().onTrue(homeElevatorArm());

        XController.leftTrigger().whileTrue(m_armSubsystem.runOnce(() -> m_armSubsystem.resetPitch()));

        // reset the field-centric heading on left bumper press
        XController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // eggYoke.button(10).toggleOnTrue(follow);

        XController.rightBumper().onTrue(intakeCoralAlgae());
        XController.rightTrigger().onTrue(m_armSubsystem.intakeOuttake(IntakeDirection.OUT));
        XController.rightTrigger().onFalse(m_armSubsystem.intakeOuttake(IntakeDirection.STOP));

        // Don't create a new command everytime it needs to be run, init at the top
        // laserCan
        // XController.x().onTrue(new InstantCommand(() ->
        // m_laserCanSubsystem.detectObject(), m_laserCanSubsystem));

        // Binding the GroundIntakeCommand
        // XController.a().onTrue(new GroundIntakeCommand(m_groundIntakeSubsystem, 45.0,
        // 90.0));
        

        // eggYoke examples for led
        // eggYoke.button(5).onTrue(new SetLedCommand(ledSubsystem,
        // RobotState.READY_TO_SHOOT));

        // drivetrain.registerTelemetry(logger::telemeterize);

        // Run SysId routines when holding 11 or 12
        // Note that each routine should be run exactly once in a single log.
        /*
         * eggYoke.button(12).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
         * eggYoke.button(11).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
         * eggYoke.button(12).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward))
         * ;
         * eggYoke.button(11).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse))
         * ;
         */
    }

    public Command getAutonomousCommand() {
        // return Commands.print("No autonomous command configured");
        // return new PathPlannerAuto("New Auto");
        return autoChooser.getSelected();
    }
}
