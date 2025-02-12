package frc.robot.commands;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;

import edu.wpi.first.wpilibj2.command.Command;

public class MusicPlayerCommand extends Command {

    private final Orchestra m_orchestra;

    public MusicPlayerCommand(Orchestra orchestra) {
        m_orchestra = orchestra;
    }

    public void initialize() {
        StatusCode loadMusicStatus = m_orchestra.loadMusic("music.chrp");
        if (loadMusicStatus.isOK()) {
            m_orchestra.play();
        } else {
            System.out.println("Failed to load CHRP music!");
        };
    }

    public void end() {

    }

    public boolean isFinished() {
        return !m_orchestra.isPlaying();
    }

    public boolean runsWhenDisabled() {
        return true;
    }

    public Command.InterruptionBehavior getInterruptionBehavior() {
        return Command.InterruptionBehavior.kCancelSelf;
    }
}
