package com.team3181.frc2023.subsystems.swerve;

import com.team3181.lib.swerve.BetterSwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
    @AutoLog
    class SwerveModuleIOInputs {
        public double drivePositionMeters = 0.0;
        public double driveVelocityMetersPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;
        public double driveTempCelcius = 0.0;

        public double steerAbsolutePositionRad = 0.0;
        public double steerAbsoluteVelocityRadPerSec = 0.0;
        public double steerPositionRad = 0.0;
        public double steerVelocityRadPerSec = 0.0;
        public double steerAppliedVolts = 0.0;
        public double steerCurrentAmps = 0.0;
        public double steerTempCelcius = 0.0;
    }

    default void updateInputs(SwerveModuleIOInputs inputs) {}

    default void setDriveVoltage(double voltage) {}

    default void setSteerVoltage(double voltage) {}
    
    default void setModuleState(BetterSwerveModuleState state, boolean isOpenLoop) {}

    default void stopMotors() {}

    default void setDriveBrakeMode(boolean enable) {}

    default void setSteerBrakeMode(boolean enable) {}
}