package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {

    private static final int LED_PORT_1 = 9;
    private static final int LED_LENGTH_1 = 53;

    private final AddressableLED m_led1;
    private final AddressableLEDBuffer m_ledBuffer1;

    private int rainbowIndex = 0;
    private double lastRainbowUpdateTime = 0;
    private static final double RAINBOW_INTERVAL = 0.5;
    private final int[][] rainbowColors = {
            { 255, 0, 0 }, { 255, 127, 0 }, { 255, 255, 0 },
            { 0, 255, 0 }, { 0, 0, 255 }, { 75, 0, 130 }, { 148, 0, 211 }
    };

    private boolean flashOn = false;
    private double lastFlashTime = 0;
    private static final double FLASH_INTERVAL = 0.5; // seconds between toggles

    private int blockPosition = 0;
    private static final int BLOCK_SIZE = 6;
    private double lastBlockUpdateTime = 0;
    private static final double BLOCK_MOVE_INTERVAL = 0.02;

    public enum RobotState {
        IDLE, RAINBOW, MOVING_BLOCK_DOWN, MOVING_BLOCK_UP, SOLID_RED, SOLID_BLUE, INTAKE_CORAL, INTAKE_BALL,
        SHOOTING_REEF, SHOOTING_BARGE, READY_TO_SHOOT,
    }

    private RobotState currentState = RobotState.IDLE;

    public LedSubsystem() {
        m_led1 = new AddressableLED(LED_PORT_1);
        m_ledBuffer1 = new AddressableLEDBuffer(LED_LENGTH_1);

        m_led1.setLength(m_ledBuffer1.getLength());
        m_led1.setData(m_ledBuffer1);
        m_led1.start();
    }

    public void setState(RobotState state) {
        this.currentState = state;
        SmartDashboard.putString("LED STATE", state.name());
        resetEffects();
    }

    private void resetEffects() {
        lastRainbowUpdateTime = Timer.getFPGATimestamp();
        rainbowIndex = 0;
        blockPosition = 0;
        resetFlashing();
    }

    private void runRainbowEffect() {
        if (Timer.getFPGATimestamp() - lastRainbowUpdateTime >= RAINBOW_INTERVAL) {
            int[] color = rainbowColors[rainbowIndex];
            setColor(color[0], color[1], color[2]);
            rainbowIndex = (rainbowIndex + 1) % rainbowColors.length;
            lastRainbowUpdateTime = Timer.getFPGATimestamp();
        }
    }

    private void runBlockEffect() {
        if (Timer.getFPGATimestamp() - lastBlockUpdateTime >= BLOCK_MOVE_INTERVAL) {
            for (int i = 0; i < m_ledBuffer1.getLength(); i++) {
                m_ledBuffer1.setRGB(i, 0, 0, 0);
            }
            for (int i = blockPosition; i < blockPosition + BLOCK_SIZE && i < m_ledBuffer1.getLength(); i++) {
                m_ledBuffer1.setRGB(i, 139, 0, 0);
            }
            m_led1.setData(m_ledBuffer1);
            blockPosition = (blockPosition + 1) % (m_ledBuffer1.getLength() - BLOCK_SIZE);
            lastBlockUpdateTime = Timer.getFPGATimestamp();
        }
    }

    private void runBlockEffect(int r, int g, int b) {
        if (Timer.getFPGATimestamp() - lastBlockUpdateTime >= BLOCK_MOVE_INTERVAL) {
            for (int i = 0; i < m_ledBuffer1.getLength(); i++) {
                m_ledBuffer1.setRGB(i, 0, 0, 0);
            }
            for (int i = blockPosition; i < blockPosition + BLOCK_SIZE && i < m_ledBuffer1.getLength(); i++) {
                m_ledBuffer1.setRGB(i, r, g, b);
            }
            m_led1.setData(m_ledBuffer1);
            blockPosition = (blockPosition + 1) % (m_ledBuffer1.getLength() - BLOCK_SIZE);
            lastBlockUpdateTime = Timer.getFPGATimestamp();
        }
    }

    private void runBlockEffectReverse() {
        if (Timer.getFPGATimestamp() - lastBlockUpdateTime >= BLOCK_MOVE_INTERVAL) {
            for (int i = 0; i < m_ledBuffer1.getLength(); i++) {
                m_ledBuffer1.setRGB(i, 0, 0, 0);
            }
            for (int i = blockPosition; i < blockPosition + BLOCK_SIZE && i < m_ledBuffer1.getLength(); i++) {
                m_ledBuffer1.setRGB(i, 139, 0, 0);
            }
            m_led1.setData(m_ledBuffer1);
            blockPosition = (blockPosition - 1) % (m_ledBuffer1.getLength() - BLOCK_SIZE);
            lastBlockUpdateTime = Timer.getFPGATimestamp();
        }
    }

    private void runBlockEffectReverse(int r, int g, int b) {
        if (Timer.getFPGATimestamp() - lastBlockUpdateTime >= BLOCK_MOVE_INTERVAL) {
            for (int i = 0; i < m_ledBuffer1.getLength(); i++) {
                m_ledBuffer1.setRGB(i, 0, 0, 0);
            }
            for (int i = blockPosition; i < blockPosition + BLOCK_SIZE && i < m_ledBuffer1.getLength(); i++) {
                m_ledBuffer1.setRGB(i, r, g, b);
            }
            m_led1.setData(m_ledBuffer1);
            blockPosition = (blockPosition - 1) % (m_ledBuffer1.getLength() - BLOCK_SIZE);
            lastBlockUpdateTime = Timer.getFPGATimestamp();
        }
    }

    private void setColor(int r, int g, int b) {
        for (int i = 0; i < m_ledBuffer1.getLength(); i++) {
            m_ledBuffer1.setRGB(i, r, g, b);
        }
        m_led1.setData(m_ledBuffer1);
    }

    private void resetFlashing() {
        flashOn = false;
        lastFlashTime = Timer.getFPGATimestamp();
    }

    private void setFlashingColor(int r, int g, int b) {
        double currentTime = Timer.getFPGATimestamp();
        if (currentTime - lastFlashTime >= FLASH_INTERVAL) {
            flashOn = !flashOn;
            lastFlashTime = currentTime;
        }

        int displayR = flashOn ? r : 0;
        int displayG = flashOn ? g : 0;
        int displayB = flashOn ? b : 0;

        for (int i = 0; i < m_ledBuffer1.getLength(); i++) {
            m_ledBuffer1.setRGB(i, displayR, displayG, displayB);
        }
        m_led1.setData(m_ledBuffer1);
    }

    @Override
    public void periodic() {
        switch (currentState) {
            case RAINBOW:
                runRainbowEffect();
                break;
            case MOVING_BLOCK_DOWN:
                runBlockEffect(); // moves down
                break;
            case MOVING_BLOCK_UP:
                runBlockEffectReverse(); // moves down
                break;
            case INTAKE_CORAL:
                setFlashingColor(255, 248, 220); // white like coral (but it's off white cuz normal white is boring)
                break;
            case INTAKE_BALL:
                setFlashingColor(100, 233, 134); // teal like algae
                break;
            case SHOOTING_REEF:
                setFlashingColor(120, 81, 169); // purple like the reef
                break;
            case SHOOTING_BARGE:
                setFlashingColor(255, 160, 122); // orange like the net
                break;
            case READY_TO_SHOOT:
                runRainbowEffect();
                setFlashingColor(255, 255, 255); // does this switch between rainbow and flash?
                break;
            case IDLE:
                setColor(139, 0, 0); // royals colour ish
                break;
            default:
                break;
        }
    }
}