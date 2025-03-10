package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {
    
    private static final int LED_PORT_1 = 9;  
    private static final int LED_PORT_2 = 10;
    private static final int LED_PORT_3 = 11;
    private static final int LED_LENGTH_1 = 27;
    private static final int LED_LENGTH_2 = 27;
    private static final int LED_LENGTH_3 = 47;

    private final AddressableLED m_led1;
    private final AddressableLEDBuffer m_ledBuffer1;
    private final AddressableLED m_led2;
    private final AddressableLEDBuffer m_ledBuffer2;
    private final AddressableLED m_led3;
    private final AddressableLEDBuffer m_ledBuffer3;

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
        
        m_led2 = new AddressableLED(LED_PORT_2);
        m_ledBuffer2 = new AddressableLEDBuffer(LED_LENGTH_2);
        m_led2.setLength(m_ledBuffer2.getLength());
        m_led2.setData(m_ledBuffer2);
        m_led2.start();
        
        m_led3 = new AddressableLED(LED_PORT_3);
        m_ledBuffer3 = new AddressableLEDBuffer(LED_LENGTH_3);
        m_led3.setLength(m_ledBuffer3.getLength());
        m_led3.setData(m_ledBuffer3);
        m_led3.start();
    }

    @Override
    public void periodic() {
        updateLEDs();
    }

    private void updateLEDs() {
        switch (currentState) {
            case INTAKE_CORAL:
                setColor(255, 140, 0); // Orange
                break;
            case INTAKE_BALL:
                setColor(0, 0, 255); // Blue
                break;
            case SHOOTING_REEF:
                setColor(128, 0, 128); // Purple
                break;
            case SHOOTING_BARGE:
                setColor(0, 255, 0); // Green
                break;
            case READY_TO_SHOOT:
                setColor(255, 255, 255); // White
                break;
            default:
                setColor(0, 0, 0); // Off
                break;
        }
    }

    private void setColor(int r, int g, int b) {
        for (int i = 0; i < m_ledBuffer1.getLength(); i++) {
            m_ledBuffer1.setRGB(i, r, g, b);
        }
        for (int i = 0; i < m_ledBuffer2.getLength(); i++) {
            m_ledBuffer2.setRGB(i, r, g, b);
        }
        for (int i = 0; i < m_ledBuffer3.getLength(); i++) {
            m_ledBuffer3.setRGB(i, r, g, b);
        }
        
        m_led1.setData(m_ledBuffer1);
        m_led2.setData(m_ledBuffer2);
        m_led3.setData(m_ledBuffer3);
    }

    public void setState(RobotState state) {
        this.currentState = state;
    }
}