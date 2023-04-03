package com.team3181.frc2023.subsystems.leds;

import com.team3181.frc2023.Constants.RobotConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class LEDs extends SubsystemBase {
    private final LEDStripIO leds;
    private final Timer timer = new Timer();
    private int initialRainbowHue = 0;

    public enum LEDModes {
        CONE, CUBE, FLASH_CONE, FLASH_CUBE, RAINBOW, GOOD, BAD, HAPPY, AUTO_NODE, AUTO_SUBSTATION, ERROR, IDLE, CONNECTED_FMS, OFF
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
            case CUBE -> setColorAll(Color.kPurple);
            case CONE -> setColorAll(Color.kYellow);
            case FLASH_CUBE -> flashColor(Color.kPurple);
            case FLASH_CONE -> flashColor(Color.kYellow);
            case CONNECTED_FMS -> setFadeAlliance();
            case GOOD -> setColorAll(Color.kGreen);
            case BAD -> setColorAll(Color.kRed);
            case AUTO_NODE -> flashColor(Color.kBlue);
            case AUTO_SUBSTATION -> flashColorBack(Color.kBlue);
            case RAINBOW -> setRainbow();
            case HAPPY -> flashColor(Color.kGreen);
            case ERROR -> flashColor(Color.kRed);
            case IDLE -> setColorAll(Color.kWhite);
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
        if (timer.get() < 0.1) {
            setColorAll(color);
        }
        else if (timer.get() < 0.2) {
            setOff();
        }
        else {
            timer.restart();
        }
    }

    private void flashColorBack(Color color) {
        if (timer.get() < 0.1) {
            setColorBack(color);
        }
        else if (timer.get() < 0.2) {
            setOffBack();
        }
        else {
            timer.restart();
        }
    }

    private void setColorAll(Color color) {
        for (int i = 0; i < leds.getLength(); i++) {
            leds.setRGB(i, (int)(color.red * 255), (int)(color.green * 255), (int)(color.blue * 255));
        }
    }

    private void setColorBack(Color color) {
        for (int i = 0; i < 10; i++) {
            leds.setRGB(i, (int)(color.red * 255), (int)(color.green * 255), (int)(color.blue * 255));
        }
        for (int i = 20; i < 30; i++) {
            leds.setRGB(i, (int)(color.red * 255), (int)(color.green * 255), (int)(color.blue * 255));
        }
    }

    private void setOffBack() {
        for (int i = 0; i < 10; i++) {
            leds.setRGB(i,0, 0,0);
        }
        for (int i = 20; i < 30; i++) {
            leds.setRGB(i,0, 0,0);
        }
    }

    private void setRunAlliance() {
        Color color = getConnected();
        if (timer.get() < 0.05 + 0.02) {
            setTwo(0, color);
        }
        else if (timer.get() < 0.1 + 0.04) {
            setTwo(1, color);
        }
        else if (timer.get() < 0.15 + 0.06) {
            setTwo(2, color);
        }
        else if (timer.get() < 0.2 + 0.08) {
            setTwo(3, color);
        }
        else if (timer.get() < 0.25 + 0.1) {
            setTwo(4, color);
        }
        else if (timer.get() < 0.3 + 0.12) {
            setTwo(5, color);
        }
        else if (timer.get() < 0.35 + 0.14) {
            setTwo(6, color);
        }
        else if (timer.get() < 0.4 + 0.16) {
            setTwo(7, color);
        }
        else if (timer.get() < 0.45 + 0.18) {
            setTwo(8, color);
        }
        else if (timer.get() < 0.5 + 0.2) {
            setTwo(7, color);
        }
        else if (timer.get() < 0.55 + 0.22) {
            setTwo(6, color);
        }
        else if (timer.get() < 0.60 + 0.24) {
            setTwo(5, color);
        }
        else if (timer.get() < 0.65 + 0.26) {
            setTwo(4, color);
        }
        else if (timer.get() < 0.7 + 0.28) {
            setTwo(3, color);
        }
        else if (timer.get() < 0.75 + 0.3) {
            setTwo(2, color);
        }
        else if (timer.get() < 0.8 + 0.32) {
            setTwo(1, color);
        }
        else if (timer.get() < 0.85 + 0.34) {
            setTwo(0, color);
        }
        else if (timer.get() < 1.15 + 0.36) {
            setOff();
        }
        else {
            timer.restart();
        }
    }

    private void setFade(Color color) {
        if (timer.get() < 2.55) {
            for (int i = 0; i < leds.getLength(); i++) {
                leds.setHSV(i, 0, 0,  (int) (100 * timer.get()));
            }
        }
        else {
            timer.restart();
        }
    }

    private Color getConnected() {
        Color color = Color.kWhite;
        if (DriverStation.getAlliance() == Alliance.Blue) {
            color = Color.kBlue;
        }
        else if (DriverStation.getAlliance() == Alliance.Red) {
            color = Color.kRed;
        }
        return color;
    }

    private void setFadeAlliance() {
        Color color = getConnected();

        if (timer.get() < 2.55) {
            for (int i = 0; i < leds.getLength(); i++) {
                float[] hsv = java.awt.Color.RGBtoHSB((int) color.red, (int) color.green, (int) color.blue, null);
                leds.setHSV(i, (int)(hsv[0] * 180), (int)(hsv[1] * 255),  (int) (100 * timer.get()));
            }
        }
        else if (timer.get() < 5.1) {
            for (int i = 0; i < leds.getLength(); i++) {
                float[] hsv = java.awt.Color.RGBtoHSB((int) color.red, (int) color.green, (int) color.blue, null);
                leds.setHSV(i, (int)(hsv[0] * 180), (int)(hsv[1] * 255), 255 - (int) (100 * (timer.get()-2.55)));
            }
        }
        else if (timer.get() < 7) {
            setOff();
        }
        else {
            timer.restart();
        }
    }

    private void setTwo(int number, Color color) {
        number = MathUtil.clamp(number, 0, 8);
        setOff();
        for (int i = 0 + number; i < 2 + number; i++) {
            leds.setRGB(i, (int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255));
        }
        for (int i = 10 + number; i < 12 + number; i++) {
            leds.setRGB(i, (int)(color.red * 255), (int)(color.green * 255), (int)(color.blue * 255));
        }
        for (int i = 20 + number; i < 22 + number; i++) {
            leds.setRGB(i, (int)(color.red * 255), (int)(color.green * 255), (int)(color.blue * 255));
        }
        for (int i = 30 + number; i < 32 + number; i++) {
            leds.setRGB(i, (int)(color.red * 255), (int)(color.green * 255), (int)(color.blue * 255));
        }
    }

    public void setLEDMode(LEDModes ledMode) {
        this.ledMode = ledMode;
    }
}