package com.team3181.frc2023.subsystems.endeffector;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.team3181.frc2023.Constants;
import com.team3181.lib.drivers.LazySparkMax;
import edu.wpi.first.math.util.Units;

public class EndEffectorIOSparkMax implements EndEffectorIO {
    private final LazySparkMax motorLeft = new LazySparkMax(Constants.EndEffectorConstants.CAN_LEFT, IdleMode.kBrake, 30, true, false);
    private final LazySparkMax motorRight = new LazySparkMax(Constants.EndEffectorConstants.CAN_RIGHT, IdleMode.kBrake, 30, motorLeft, true, false);
    private final RelativeEncoder encoder = motorLeft.getEncoder();

    public EndEffectorIOSparkMax() {
        encoder.setPositionConversionFactor(Constants.EndEffectorConstants.GEARING);
        encoder.setVelocityConversionFactor(Constants.EndEffectorConstants.GEARING / 60.0);
        motorLeft.burnFlash();
    }

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        inputs.positionRad = encoder.getPosition();
        inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity());
        inputs.appliedVolts = motorLeft.getAppliedOutput() * motorLeft.getBusVoltage();
        inputs.currentAmps = motorLeft.getOutputCurrent();
        inputs.tempCelsius = motorLeft.getMotorTemperature();
    }

    @Override
    public void setVoltage(double volts) {
        motorLeft.setVoltage(volts);
    }

    @Override
    public void setBrakeMode(boolean enable) {
        motorLeft.setIdleMode(enable ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
    }
}