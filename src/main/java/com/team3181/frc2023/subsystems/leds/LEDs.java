package com.team3181.frc2023.subsystems.leds;

import com.team3181.frc2023.Constants.RobotConstants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class LEDs extends SubsystemBase {
    private final LEDStripIO leds;
    private final Timer timer = new Timer();
    private int initialRainbowHue = 0;

    public enum LEDModes {
        CONE, CUBE, FLASH_CONE, FLASH_CUBE, RAINBOW, GOOD, BAD, HAPPY, ERROR, IDLE
    }
    private LEDModes ledMode = LEDModes.IDLE;

    private final static LEDs INSTANCE = new LEDs(RobotConstants.LEDS);

    public static LEDs getInstance() {
        return INSTANCE;
    }
    private LEDs(LEDStripIO leds) {
        this.leds = leds;
        timer.start();
    }

    @Override
    public void periodic() {
        switch (ledMode) {
            case CUBE -> setColor(Color.kPurple);
            case CONE -> setColor(Color.kYellow);
            case FLASH_CUBE -> flashColor(Color.kPurple);
            case FLASH_CONE -> flashColor(Color.kYellow);
            case GOOD -> setColor(Color.kGreen);
            case BAD -> setColor(Color.kRed);
            case RAINBOW -> setRainbow();
            case HAPPY -> flashColor(Color.kGreen);
            case ERROR -> flashColor(Color.kRed);
            default -> setOff();
        }
        leds.setBuffer();

        Logger.getInstance().recordOutput("LEDS/Current Mode", ledMode.toString());
    }

    private void setOff() {
        for (int i = 0; i < leds.getLength(); i++) {
            leds.setRGB(i, 0, 0, 0);
        }
    }

    private void setRainbow() {
        for (var i = 0; i < leds.getLength(); i++) {
            final int hue = (initialRainbowHue + (i * 180 / leds.getLength())) % 180;
            leds.setHSV(i, hue, 255, 128);
        }
        initialRainbowHue += 3;
        initialRainbowHue %= 180;
    }

    private void flashColor(Color color) {
        if (timer.get() < 0.5) {
            setColor(color);
        }
        else if (timer.get() < 1) {
            setOff();
        }
        else {
            timer.restart();
        }
    }

    private void setColor(Color color) {
        for (int i = 0; i < leds.getLength(); i++) {
            leds.setRGB(i, (int)(color.red * 255), (int)(color.green * 255), (int)(color.blue * 255));
        }
    }

    public void setLEDMode(LEDModes ledMode) {
        this.ledMode = ledMode;
    }
}