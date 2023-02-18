package com.team3181.frc2023.subsystems.fourbar;

import org.littletonrobotics.junction.AutoLog;

public class FourBarIO {
    @AutoLog
    class FourBarIOInputs{
        public double shoulderPositionRad = 0.0;
        public double elbowPositionRad = 0.0;
        public double shoulderVelocityRadPerSec = 0.0;
        public double elbowVelocityRadPerSec = 0.0;

        public double appliedVolts = 0.0;
        // public double[] currentAmps = new double[] {};
        // public double[] tempCelsius = new double[] {};
    }
    default void updateInputs(FourBarIOInputs inputs) {}

    default void setVoltage(double volts) {}

    default void setBrakeMode(boolean enable) {}
}
