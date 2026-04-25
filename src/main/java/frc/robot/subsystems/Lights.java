package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import java.util.Optional;

public class Lights {
    private Optional<Alliance> alliance = DriverStation.getAlliance();
    private Color idleColor = Color.kRed;
    final LEDPattern aimingPattern = LEDPattern.solid(Color.kOrange);
    final LEDPattern shootingPattern = LEDPattern.solid(Color.kGreen);
    AddressableLED m_led = new AddressableLED(9);
    AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(60);

    public void initialize() {
        m_led.setLength(m_ledBuffer.getLength());

        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    public void idle() {
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            idleColor = Color.kRed;
        } else {
            idleColor = Color.kBlue;
        }

        final LEDPattern idlePattern = LEDPattern.solid(idleColor);

        idlePattern.applyTo(m_ledBuffer);
        m_led.setData(m_ledBuffer);
    }

    public void aiming() {
        aimingPattern.applyTo(m_ledBuffer);
        m_led.setData(m_ledBuffer);
    }

    public void shooting() {
        shootingPattern.applyTo(m_ledBuffer);
        m_led.setData(m_ledBuffer);
    }
}
