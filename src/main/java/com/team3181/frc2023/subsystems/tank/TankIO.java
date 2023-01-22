package com.team3181.frc2023.subsystems.tank;

import org.littletonrobotics.junction.AutoLog;

/** Tank subsystem hardware interface. */
public interface TankIO {
    /** The set of loggable inputs for the drive subsystem. */
    @AutoLog
    class DriveIOInputs {
        public double leftPositionMeters = 0.0;
        public double leftVelocityMetersPerSec = 0.0;
        public double leftAppliedVolts = 0.0;
        public double[] leftCurrentAmps = new double[] {};
        public double[] leftTempCelsius = new double[] {};

        public double rightPositionMeters = 0.0;
        public double rightVelocityMetersPerSec = 0.0;
        public double rightAppliedVolts = 0.0;
        public double[] rightCurrentAmps = new double[] {};
        public double[] rightTempCelsius = new double[] {};

        public boolean gyroConnected = false;
        public double gyroYawPositionRad = 0.0;
        public double gyroYawVelocityRadPerSec = 0.0;
        public double gyroPitchPositionRad = 0.0;
        public double gyroPitchVelocityRadPerSec = 0.0;
        public double gyroRollPositionRad = 0.0;
        public double gyroRollVelocityRadPerSec = 0.0;
    }

    /** Updates the set of loggable inputs. */
    default void updateInputs(DriveIOInputs inputs) {}

    /** Run open loop at the percentage of 12V from -1.0 to 1.0. */
    default void set(double leftPercent, double rightPercent) {}

    /** Run open loop at the specified voltage. */
    default void setVoltage(double leftVolts, double rightVolts) {}

    /** Enable or disable brake mode. */
    default void setBrakeMode(boolean enable) {}

    /** Resets encoders to 0 */
    default void resetEncoders() {}
}