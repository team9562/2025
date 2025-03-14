// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
import frc.robot.commands.ArmCommands.HomeArm;
import frc.robot.commands.ArmCommands.IntakeOutake;
import frc.robot.commands.ArmCommands.SetAngleToPOI;
import frc.robot.commands.ElevatorCommands.HomeElevator;
import frc.robot.commands.ElevatorCommands.SetHeightToPOI;
import frc.robot.commands.IntakeCommands.GroundIntakeCommand;
import frc.robot.commands.LEDCommands.SetLedCommand;
import frc.robot.commands.SwerveCommands.GoToBestTargetCommand;
import frc.robot.commands.SwerveCommands.TurnAroundCommand;
import frc.robot.commands.SwerveCommands.TurnToBestTargetCommand;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem;
import frc.robot.subsystems.LaserCanSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.LedSubsystem.RobotState;

public class RobotContainer {

    // speed is divided by 3 to accommodate for small testing spaces
    public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                        // speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second -
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.2).withRotationalDeadband(MaxAngularRate * 0.3) // Adds a 20% deadband to the
                                                                                       // controller - UPDATED -
                                                                                       // ROTATIONAL DEADBAND TO 30
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    // LazerCan
    public final LaserCanSubsystem m_laserCanSubsystem = new LaserCanSubsystem();

    //private final Telemetry logger = new Telemetry(MaxSpeed);

    static CommandXboxController XController = new CommandXboxController(0);

    public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final static ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
    public final static VisionSubsystem m_visionSubsystem = new VisionSubsystem();
    public final static ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
    public final static GroundIntakeSubsystem m_groundIntakeSubsystem = new GroundIntakeSubsystem();
    public final LedSubsystem ledSubsystem = new LedSubsystem();

    private final SendableChooser<Command> autoChooser;

    private final Command turnAroundCommand = new TurnAroundCommand(drivetrain, drive, MaxAngularRate);

    private final Command turnToBestTargetCommand = new TurnToBestTargetCommand(drivetrain, m_visionSubsystem, drive, 0);

    private final Command goToBestTargetCommand = new GoToBestTargetCommand(drivetrain, m_visionSubsystem, drive);
    public RobotContainer() {
        registerCommands();
        burnAllFlash();
        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser("New New Auto");
        SmartDashboard.putData("Auto chooser", autoChooser);
    }

    private void registerCommands() {
        NamedCommands.registerCommand("turnAround", turnAroundCommand);
        NamedCommands.registerCommand("turnToBestTarget", turnToBestTargetCommand);
        NamedCommands.registerCommand("goToBestTargetCommand", goToBestTargetCommand);
    }

    private void burnAllFlash() {
        m_elevatorSubsystem.burnFlash();
        m_ArmSubsystem.burnFlash();
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
                        .withRotationalRate(XController.getRightX() * MaxAngularRate)));

        // try to get this to work properly, might need to convert into a command in the
        // subsystem itself
        m_ArmSubsystem.setDefaultCommand(
                m_ArmSubsystem.run(() -> m_ArmSubsystem.manualPitchMotor(XController.getRightY())));

        // new awesome code - yay
        // might work better like instead of intializing four at the top: control via
        // d-pad
        XController.povUp().onTrue(new SetHeightToPOI(m_elevatorSubsystem, "l4"));    // 67.17 in
        XController.povLeft().onTrue(new SetHeightToPOI(m_elevatorSubsystem, "l3"));  // 43.86 in
        XController.povRight().onTrue(new SetHeightToPOI(m_elevatorSubsystem, "l2")); // 26.85 in
        XController.povDown().onTrue(new SetHeightToPOI(m_elevatorSubsystem, "b"));   // 77.1 in 

        XController.y().onTrue(goToBestTargetCommand); // no exit command rn -> fix later
        XController.rightBumper().onTrue(m_elevatorSubsystem.runCurrentZeroing());
        XController.leftStick().onTrue(new SetAngleToPOI(m_ArmSubsystem, "l2"));

        XController.y().onTrue(turnToBestTargetCommand); // no exit command rn -> fix later
        XController.rightStick().onTrue(m_elevatorSubsystem.runCurrentZeroing());

        XController.leftTrigger().whileTrue(m_ArmSubsystem.run(() -> m_ArmSubsystem.resetPitch()));

        // reset the field-centric heading on left bumper press
        XController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // eggYoke.button(10).toggleOnTrue(follow);

        XController.rightBumper().onTrue(new IntakeOutake(m_ArmSubsystem, "in"));
        XController.rightTrigger().onTrue(new IntakeOutake(m_ArmSubsystem, "out"));

        // Don't create a new command everytime it needs to be run, init at the top
        // laserCan
        // XController.x().onTrue(new InstantCommand(() -> m_laserCanSubsystem.detectObject(), m_laserCanSubsystem));

        // Binding the GroundIntakeCommand
        XController.a().onTrue(new GroundIntakeCommand(m_groundIntakeSubsystem, 45.0, 90.0));
        XController.b().onTrue(new InstantCommand(() -> ledSubsystem.applyBlockEffect()));

        // eggYoke examples for led
        // eggYoke.button(5).onTrue(new SetLedCommand(ledSubsystem,
        // RobotState.READY_TO_SHOOT));

        //drivetrain.registerTelemetry(logger::telemeterize);

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
        //return Commands.print("No autonomous command configured");
        //return new PathPlannerAuto("New Auto");
        return autoChooser.getSelected();
    }
}
