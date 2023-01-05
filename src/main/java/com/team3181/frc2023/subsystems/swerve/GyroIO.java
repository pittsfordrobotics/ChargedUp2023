package com.team3181.frc2023.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    class GyroIOInputs {
        public boolean connected = false;
        public double yawPositionRad = 0.0;
        public double yawVelocityRadPerSec = 0.0;
        public double pitchPositionRad = 0.0;
        public double rollPositionRad = 0.0;
    }

    default void updateInputs(GyroIOInputs inputs) {}
}