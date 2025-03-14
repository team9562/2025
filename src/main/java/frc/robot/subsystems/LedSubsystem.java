package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;

public class LedSubsystem extends SubsystemBase {

    private static final int LED_PORT_1 = 9; // Using only one LED strip on port 7
    private static final int LED_LENGTH_1 = 53; // Length of the LED strip
    private static final int BLOCK_SIZE = 6;  // Size of the red block (6 LEDs)

    private final AddressableLED m_led1;
    private final AddressableLEDBuffer m_ledBuffer1;

    // Rainbow effect variables
    private int rainbowIndex = 0;
    private double lastRainbowUpdateTime = 0;
    private static final double RAINBOW_COLOR_CHANGE_INTERVAL = 0.5;
    private final int[][] rainbowColors = {
            {255, 0, 0},    // Red
            {255, 127, 0},  // Orange
            {255, 255, 0},  // Yellow
            {0, 255, 0},    // Green
            {0, 0, 255},    // Blue
            {75, 0, 130},   // Indigo
            {148, 0, 211}   // Violet
    };

    // Block effect variables
    private int blockPosition = 0;
    private double lastBlockUpdateTime = 0;
    private static final double BLOCK_MOVE_INTERVAL = 0.02;

    // Robot state enum
    public enum RobotState {
        IDLE,
        INTAKE_CORAL,
        INTAKE_BALL,
        SHOOTING_REEF,
        SHOOTING_BARGE,
        READY_TO_SHOOT
    }

    private RobotState currentState = RobotState.IDLE;

    public LedSubsystem() {
        m_led1 = new AddressableLED(LED_PORT_1);
        m_ledBuffer1 = new AddressableLEDBuffer(LED_LENGTH_1);

        m_led1.setLength(m_ledBuffer1.getLength());
        m_led1.setData(m_ledBuffer1);
        m_led1.start();
    }

    // Method to start the rainbow effect
    public void applyRainbowEffect() {
        lastRainbowUpdateTime = Timer.getFPGATimestamp(); // Start the timer for periodic color change
        rainbowIndex = 0; // Reset rainbow color index
    }

    private void runRainbowEffect() {
        // This function will be periodically called to update the LED colors
        if (Timer.getFPGATimestamp() - lastRainbowUpdateTime >= RAINBOW_COLOR_CHANGE_INTERVAL) {
            // Cycle through rainbow colors
            int[] currentColor = rainbowColors[rainbowIndex];

            // Set the color for the LED strip
            setColor(m_ledBuffer1, currentColor[0], currentColor[1], currentColor[2]);

            // Increment index to show next color, wrap around at the end
            rainbowIndex = (rainbowIndex + 1) % rainbowColors.length;

            // Update the LED strip with the new color data
            m_led1.setData(m_ledBuffer1);

            // Reset the timer for the next update
            lastRainbowUpdateTime = Timer.getFPGATimestamp();
        }
    }

    // Method to start the block effect
    public void applyBlockEffect() {
        lastBlockUpdateTime = Timer.getFPGATimestamp(); // Start the timer for block movement
        blockPosition = 0; // Reset block position to start at the bottom
    }

    private void runBlockEffect() {
        // This function will be periodically called to update the LED colors
        if (Timer.getFPGATimestamp() - lastBlockUpdateTime >= BLOCK_MOVE_INTERVAL) {
            // Clear all LEDs (turn off the strip)
            for (int i = 0; i < m_ledBuffer1.getLength(); i++) {
                m_ledBuffer1.setRGB(i, 0, 0, 0);  // Set all LEDs to off
            }

            // Turn on the block of LEDs in red
            for (int i = blockPosition; i < blockPosition + BLOCK_SIZE && i < m_ledBuffer1.getLength(); i++) {
                m_ledBuffer1.setRGB(i, 0, 0, 255);  // Set LEDs to red
            }
            // Update the LED strip with the new color data
            m_led1.setData(m_ledBuffer1);

            // Update the block's position: move it up or down
            blockPosition = (blockPosition + 1) % (m_ledBuffer1.getLength() - BLOCK_SIZE);

            // Reset the timer for the next update
            lastBlockUpdateTime = Timer.getFPGATimestamp();
        }
    }

    // Utility method to set color for a given buffer
    private void setColor(AddressableLEDBuffer buffer, int r, int g, int b) {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, r, g, b);
        }
    }

    // Override the periodic method to ensure periodic LED updates
    @Override
    public void periodic() {
        // Call the rainbow effect function periodically
        // runRainbowEffect();

        // Call the block effect function periodically
        runBlockEffect();
    }

    // Method to set the robot's state (which will update the LEDs based on the state)
    public void setState(RobotState state) {
        this.currentState = state;
    }
}