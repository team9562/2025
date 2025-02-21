package frc.robot.commands;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;

public class MusicPlayerCommand extends Command {

    private final Orchestra m_orchestra;
    private String m_soundName;

    public MusicPlayerCommand(Orchestra orchestra) {
        m_orchestra = orchestra;
        m_soundName = "angry_birds";
    }

    public MusicPlayerCommand(Orchestra orchestra, String soundName) {
        m_orchestra = orchestra;
        m_soundName = soundName;
    }

    public MusicPlayerCommand withSoundName(String soundName) {
        m_soundName = soundName;
        return this;
    }

    public void initialize() {
        StatusCode loadMusicStatus = m_orchestra.loadMusic(Filesystem.getDeployDirectory()+"/music/"+m_soundName+".chrp");
        if (loadMusicStatus.isOK()) {
            m_orchestra.play();
        } else {
            System.out.println("Failed to load CHRP music!");
        };
    }

    public void end() {
        m_orchestra.stop();
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
