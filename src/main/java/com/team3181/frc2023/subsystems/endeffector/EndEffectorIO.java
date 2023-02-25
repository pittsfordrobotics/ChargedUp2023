package com.team3181.frc2023.subsystems.endeffector;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {
    @AutoLog
    class EndEffectorIOInputs {
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double tempCelsius = 0.0;
    }

    // override in iosparkmax
    default void updateInputs(EndEffectorIOInputs inputs) {}

    default void setVoltage(double volts) {}

    default void setBrakeMode(boolean enable) {}
}