package com.team3181.frc2023.subsystems.fourbar;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.team3181.frc2023.Constants.FourBarConstants;
import com.team3181.lib.drivers.LazySparkMax;
import edu.wpi.first.math.util.Units;

public class ArmIOElbowSparkMax implements ArmIO {
    private final LazySparkMax motor;
    private final AbsoluteEncoder absoluteEncoder;

    public ArmIOElbowSparkMax() {
        motor = new LazySparkMax(FourBarConstants.CAN_ELBOW, IdleMode.kBrake, 80, false,false);
        absoluteEncoder = motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        SparkMaxPIDController pidController = motor.getPIDController();

        absoluteEncoder.setInverted(true);
        absoluteEncoder.setPositionConversionFactor(2 * Math.PI * FourBarConstants.CHAIN_RATIO);
        absoluteEncoder.setVelocityConversionFactor(2 * Math.PI * FourBarConstants.CHAIN_RATIO / 60.0);
        absoluteEncoder.setZeroOffset(FourBarConstants.ELBOW_ABSOLUTE_OFFSET.getRadians());

        pidController.setP(FourBarConstants.ELBOW_P);
        pidController.setI(FourBarConstants.ELBOW_I);
        pidController.setD(FourBarConstants.ELBOW_D);
        pidController.setFeedbackDevice(absoluteEncoder);
        pidController.setPositionPIDWrappingMinInput(0);
        pidController.setPositionPIDWrappingMaxInput(2 * Math.PI / FourBarConstants.CHAIN_RATIO);
        pidController.setPositionPIDWrappingEnabled(true);

        motor.burnFlash();
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.armPositionRad = absoluteEncoder.getPosition() + FourBarConstants.ELBOW_MATH_OFFSET.getRadians();
        inputs.armVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(absoluteEncoder.getVelocity());
        inputs.armAppliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.armCurrentAmps = motor.getOutputCurrent();
        inputs.armTempCelsius = motor.getMotorTemperature();
    }

    @Override
    public void setVoltage(double volts) {
//        if (absoluteEncoder.getPosition() < FourBarConstants.ELBOW_MAX.getRadians() && absoluteEncoder.getPosition() > FourBarConstants.ELBOW_MIN.getRadians()) {
            motor.setVoltage(volts);
//        }
    }

    @Override
    public void setPosition(double volts) {
        motor.setVoltage(volts);
    }


    @Override
    public void setBrakeMode(boolean enable) {
        motor.setIdleMode(enable ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
    }
}