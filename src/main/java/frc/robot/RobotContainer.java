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
import frc.robot.commands.followGuzPath;
import frc.robot.commands.LEDCommands.SetLedCommand;
import frc.robot.commands.SwerveCommands.TurnAroundCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LaserCanSubsystem;
import frc.robot.subsystems.LedSubsystem;
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

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private static CommandJoystick eggYoke = new CommandJoystick(0);
    static CommandXboxController XController = new CommandXboxController(1);

    public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
    public final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
    // public final LedSubsystem ledSubsystem = new LedSubsystem();

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

        XController.a().onTrue(m_elevatorSubsystem.setElevatorHeight(67));
        XController.rightBumper().onTrue(m_elevatorSubsystem.runCurrentZeroing());

        eggYoke.button(7).onTrue(turnAroundCommand);

        eggYoke.button(6).whileTrue(drivetrain.applyRequest(() -> brake));

        eggYoke.button(4).whileTrue(drivetrain
                .applyRequest(() -> point.withModuleDirection(new Rotation2d(-eggYoke.getY(), -eggYoke.getX()))));

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

        // reset the field-centric heading on left bumper press
        eggYoke.button(2).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // eggYoke.button(10).toggleOnTrue(follow);

        XController.b().whileTrue(m_ArmSubsystem.run(() -> m_ArmSubsystem.turnOpenMotor(1)));
        XController.b().whileFalse(m_ArmSubsystem.run(() -> m_ArmSubsystem.turnOpenMotor(0)));

        // Don't create a new command everytime it needs to be run, init at the top
        // laserCan
        eggYoke.button(12).onTrue(new InstantCommand(() -> m_laserCanSubsystem.detectObject(), m_laserCanSubsystem));

        // eggYoke examples for led
        // eggYoke.button(5).onTrue(new SetLedCommand(ledSubsystem,
        // RobotState.READY_TO_SHOOT));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // return Commands.print("No autonomous command configured");
        return new PathPlannerAuto("New Auto");
    }
}
