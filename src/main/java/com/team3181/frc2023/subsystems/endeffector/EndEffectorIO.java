package com.team3181.frc2023.subsystems.endeffector;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface EndEffectorIO {
    @AutoLog
    class EndEffectorIOInputs {
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double tempCelcius = 0.0;
    }

    // override in iosparkmax
    default void updateInputs(EndEffectorIOInputs inputs) {}

    default void setVoltage(double volts) {}

    default void setBrakeMode(boolean enable) {}
}