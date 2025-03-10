// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.TurnAroundCommand;
import frc.robot.commands.followGuzPath;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LaserCanSubsystem;

public class RobotContainer {

    // speed is divided by 3 to accommodate for small testing spaces
    public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second - max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.2).withRotationalDeadband(MaxAngularRate * 0.3) // Adds a 20% deadband to the controller - UPDATED - ROTATIONAL DEADBAND TO 30
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    // LazerCan
    public final LaserCanSubsystem m_laserCanSubsystem = new LaserCanSubsystem();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private static CommandJoystick eggYoke = new CommandJoystick(0);
    static CommandXboxController XController = new CommandXboxController(0);

    public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
    public final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();

    private final Command turnAroundCommand = new TurnAroundCommand(drivetrain, drive, MaxAngularRate);

    public static final Command follow = new followGuzPath(drivetrain, eggYoke);

    public RobotContainer() {
        registerCommands();
        burnAllFlash();
        configureBindings();
    }

    private void registerCommands() {
        NamedCommands.registerCommand("turnAround", turnAroundCommand);
    }

    private void burnAllFlash() {
        m_elevatorSubsystem.burnFlash();
        m_ArmSubsystem.burnFlash();
        // intake
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(eggYoke.getY() * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(eggYoke.getX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-eggYoke.getZ() * MaxAngularRate)));

        /*m_elevatorSubsystem.setDefaultCommand(
                m_elevatorSubsystem.run(() -> this.m_elevatorSubsystem.moveElevator(eggYoke.getY() * 3))
                        .onlyWhile(() -> Math.abs(Math.round(eggYoke.getY())) != 0)
                        .finallyDo(() -> m_elevatorSubsystem.stopElevator()));*/

        eggYoke.button(7).onTrue(turnAroundCommand);

        // elevator command stuff
        eggYoke.button(3).onChange(m_elevatorSubsystem.runCurrentZeroing());

        eggYoke.button(6).whileTrue(drivetrain.applyRequest(() -> brake));

        eggYoke.button(4).whileTrue(drivetrain
                .applyRequest(() -> point.withModuleDirection(new Rotation2d(-eggYoke.getY(), -eggYoke.getX()))));

        // Run SysId routines when holding 11 or 12
        // Note that each routine should be run exactly once in a single log.
        /*eggYoke.button(12).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        eggYoke.button(11).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        eggYoke.button(12).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        eggYoke.button(11).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));*/

        // reset the field-centric heading on left bumper press
        eggYoke.button(2).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        //eggYoke.button(10).toggleOnTrue(follow);

        eggYoke.button(8).onTrue(m_elevatorSubsystem.run(() -> m_elevatorSubsystem.setElevatorHeight(20)));

        eggYoke.button(9).whileTrue(m_ArmSubsystem.turnOpenMotor(1));
        eggYoke.button(10).whileTrue(m_ArmSubsystem.turnOpenMotor(-1));
        eggYoke.button(11).onChange(m_ArmSubsystem.turnOpenMotor(0));

        //laserCan
        eggYoke.button(12).onTrue(new InstantCommand(() -> m_laserCanSubsystem.detectObject(), m_laserCanSubsystem));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // return Commands.print("No autonomous command configured");
        return new PathPlannerAuto("New Auto");
    }
}
