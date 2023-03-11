package com.team3181.frc2023.subsystems.fourbar;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    class ArmIOInputs{
        public double armPositionRad = 0.0;
        public double armVelocityRadPerSec = 0.0;

        public double armAppliedVolts = 0.0;
        public double armCurrentAmps = 0.0;
        public double armTempCelsius = 0.0;
    }
    default void updateInputs(ArmIOInputs inputs) {}

    default void setVoltage(double volts) {}

    default void setBrakeMode(boolean enable) {}
}