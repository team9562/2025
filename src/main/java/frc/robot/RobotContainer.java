// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.turnAround;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {

    //speed is divided by 3 to accommodate for small testing spaces
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) / 3; // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.2).withRotationalDeadband(MaxAngularRate * 0.3) // Adds a 20% deadband to the controller
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandJoystick eggYoke = new CommandJoystick(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final Command turnAround = new turnAround(drivetrain, drive, MaxAngularRate);

    public RobotContainer() {

        NamedCommands.registerCommand("turnAround", turnAround);
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(eggYoke.getY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(eggYoke.getX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-eggYoke.getZ() * MaxAngularRate)
            )
        );

        //drivetrain.setDefaultCommand(turnAround);
        eggYoke.button(7).onTrue(turnAround);

        //code (only useful for testing purposes) that can isolate steering and driving
        eggYoke.button(5).whileTrue(drivetrain.applyRequest(() -> drive.withRotationalRate(-eggYoke.getZ() * MaxAngularRate)));
        eggYoke.button(3).whileTrue(drivetrain.applyRequest(() -> 
            drive.withVelocityX(eggYoke.getY() * MaxSpeed)
                .withVelocityY(eggYoke.getX() * MaxSpeed)
            )
        );

        eggYoke.button(6).whileTrue(drivetrain.applyRequest(() -> brake));

        eggYoke.button(4).whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-eggYoke.getY(), -eggYoke.getX()))
        ));

        // Run SysId routines when holding 11 or 12
        // Note that each routine should be run exactly once in a single log.
        eggYoke.button(12).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        eggYoke.button(11).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        eggYoke.button(12).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        eggYoke.button(11).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        eggYoke.button(2).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
