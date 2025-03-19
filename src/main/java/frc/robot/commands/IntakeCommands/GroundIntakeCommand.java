package frc.robot.commands.IntakeCommands;

import frc.robot.subsystems.GroundIntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class GroundIntakeCommand extends Command {
    private final GroundIntakeSubsystem intakeSubsystem;
    private final double deltaAngle;

    public GroundIntakeCommand(GroundIntakeSubsystem intakeSubsystem, double deltaAngle) {
        this.intakeSubsystem = intakeSubsystem;
        
        // Store the delta angle (the amount to move from current position)
        this.deltaAngle = deltaAngle;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        // Get the current position and apply the delta angle to it
        double currentAngle = intakeSubsystem.getIntakePosition();
        double targetAngle = currentAngle + deltaAngle; // Move by delta (positive or negative)
        
        // Set the intake position to the new target angle
        intakeSubsystem.setIntakePosition(targetAngle);
    }

    @Override
    public void execute() {
        // The arm will continue moving towards the new target position
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the motor once the command ends
        intakeSubsystem.stopRotation();
    }

    @Override
    public boolean isFinished() {
        // Command finishes once the intake arm reaches the new target position
        return intakeSubsystem.isAtTarget();
    }
}
