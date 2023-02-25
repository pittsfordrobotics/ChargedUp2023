package com.team3181.frc2023.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDStripIORio implements LEDStripIO {
    private final AddressableLED ledStrip;
    private final AddressableLEDBuffer ledBuffer;

    public LEDStripIORio(int pwmPort, int ledNumber) {
        ledStrip = new AddressableLED(pwmPort);
        ledBuffer = new AddressableLEDBuffer(ledNumber);
        ledStrip.setLength(ledBuffer.getLength());
        ledStrip.setData(ledBuffer);
        ledStrip.start();
    }

    @Override
    public int getLength() {
        return ledBuffer.getLength();
    }

    @Override
    public void setRGB(int i, int r, int g, int b) {
        ledBuffer.setRGB(i, r, g, b);
    }

    @Override
    public void setHSV(int i, int h, int s, int v) {
        ledBuffer.setHSV(i, h, s, v);
    }

    @Override
    public void setBuffer() {
        ledStrip.setData(ledBuffer);
    }
}