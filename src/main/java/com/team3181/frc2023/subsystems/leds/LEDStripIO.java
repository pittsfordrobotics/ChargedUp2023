package com.team3181.frc2023.subsystems.leds;

public interface LEDStripIO {

    default int getLength() {
        return 0;
    }

    default void setRGB(int i, int r, int g, int b) {}

    default void setHSV(int i, int h, int s, int v) {}

    default void setBuffer() {}
}