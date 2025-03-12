package frc.robot.commands;

import frc.robot.subsystems.GroundIntakeSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class GroundIntakeCommand extends Command {
    private final GroundIntakeSubsystem intakeSubsystem;
    private final double targetAngle;
    private final double openAngle;

    private final Timer timer = new Timer();
    private int stage = 0;

    // Timing constants (in seconds)
    private static final double PICKUP_TIME = 1.0; // Time to run intake for pickup
    private static final double THROW_TIME = 0.5; // Time to run intake in reverse to throw ball

    public GroundIntakeCommand(GroundIntakeSubsystem intakeSubsystem, double openAngle, double targetAngle) {
        this.intakeSubsystem = intakeSubsystem;
        this.openAngle = openAngle;
        this.targetAngle = targetAngle;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        stage = 0;
        timer.reset();
        timer.start();

        // Stage 0: Set the intake to the open (pickup) position and run the intake motor to pick up the ball.
        intakeSubsystem.setIntakePosition(openAngle);
        intakeSubsystem.runIntake(0.5); // Adjust speed as necessary for pickup
    }

    @Override
    public void execute() {
        switch (stage) {
            case 0:
                // Phase: Pickup the ball
                if (timer.get() >= PICKUP_TIME) {
                    intakeSubsystem.stopIntake();
                    // After picking up, move to the target angle
                    intakeSubsystem.setIntakePosition(targetAngle);
                    stage = 1;
                }
                break;

            case 1:
                // Phase: Wait for the intake mechanism to reach the target angle
                if (intakeSubsystem.isAtTarget()) {
                    // Once at target, run the intake motor in reverse to throw the ball
                    intakeSubsystem.runIntake(-0.5); // Reverse speed to throw the ball out
                    timer.reset();
                    stage = 2;
                }
                break;

            case 2:
                // Phase: Throw the ball out
                if (timer.get() >= THROW_TIME) {
                    intakeSubsystem.stopIntake();
                    intakeSubsystem.setIntakePosition(0.0); // Retract the intake
                    stage = 3;
                }
                break;

            case 3:
                // Final phase: Ensure everything is stopped
                break;

            default:
                break;
        }
    }

    @Override
    public boolean isFinished() {
        // Command is finished when the intake is retracted (stage 3) and the target (closed position) is reached
        return stage == 3 && intakeSubsystem.isAtTarget();
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the intake motor and reset the position in case the command is interrupted
        intakeSubsystem.stopIntake();
    }
}