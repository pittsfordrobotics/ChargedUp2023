package com.team3181.frc2023.subsystems.fourbar;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.team3181.lib.drivers.LazySparkMax;
import edu.wpi.first.math.util.Units;

public class ArmIOSparkMax implements ArmIO {
    public final LazySparkMax motor;
    public final AbsoluteEncoder absoluteEncoder;

    public ArmIOSparkMax(int id) {
        motor = new LazySparkMax(id, CANSparkMax.IdleMode.kBrake, 30);
        absoluteEncoder = motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.armPositionRad = absoluteEncoder.getPosition();
        inputs.armVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(absoluteEncoder.getVelocity());
        inputs.armAppliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.armCurrentAmps = motor.getOutputCurrent();
        inputs.armTempCelsius = motor.getMotorTemperature();
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
