package frc.robot.commands.LEDCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.LedSubsystem.RobotState;

public class SetLedStateCommand extends InstantCommand {
    private final LedSubsystem ledSubsystem;
    private final RobotState targetState;

    public SetLedStateCommand(LedSubsystem ledSubsystem, RobotState targetState) {
        this.ledSubsystem = ledSubsystem;
        this.targetState = targetState;
        addRequirements(ledSubsystem); // Ensure this command requires the LED subsystem
    }

    @Override
    public void initialize() {
        ledSubsystem.setState(targetState); // Set the LED state to the target state
    }
}

/*
 
 To call this command do the the following:

 - Add these libraries to the top of the code:
    import frc.robot.subsystems.LedSubsystem;
    import frc.robot.commands.LEDCommands.SetLedStateCommand;
    import frc.robot.subsystems.LedSubsystem.RobotState;

- Initalize/define it --> private final LedSubsystem ledSubsystem;

- Call this in initalize part --> new SetLedStateCommand(ledSubsystem, RobotState.RAINBOW).schedule();
- Call this in end part --> new SetLedStateCommand(ledSubsystem, RobotState.IDLE).schedule();

 */
