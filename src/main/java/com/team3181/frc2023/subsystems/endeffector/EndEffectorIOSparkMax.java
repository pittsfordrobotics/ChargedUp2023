package com.team3181.frc2023.subsystems.endeffector;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.team3181.frc2023.Constants;
import com.team3181.lib.drivers.LazySparkMax;
import edu.wpi.first.math.util.Units;

public class EndEffectorIOSparkMax implements EndEffectorIO {
    private final LazySparkMax motor = new LazySparkMax(Constants.EndEffectorConstants.INTAKE_CAN_MAIN, CANSparkMax.IdleMode.kBrake, 30, true, true); // currentlimit stolen from 2022 intake
    private final RelativeEncoder encoder = motor.getEncoder();

    public EndEffectorIOSparkMax() {
        encoder.setPositionConversionFactor(Constants.EndEffectorConstants.GEARING);
        encoder.setVelocityConversionFactor(Constants.EndEffectorConstants.GEARING / 60.0);
    }

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        inputs.positionRad = encoder.getPosition();
        inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity());
        inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.currentAmps = motor.getOutputCurrent();
        inputs.tempCelsius = motor.getMotorTemperature();
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void setBrakeMode(boolean enable) {
        motor.setIdleMode(enable ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
    }
}