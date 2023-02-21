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
    private final SparkMaxPIDController pidController;
    private double lastPos = 0;
    private double counter = 0;

    public ArmIOElbowSparkMax() {
        motor = new LazySparkMax(FourBarConstants.CAN_ELBOW, IdleMode.kBrake, 80, false,false);
        absoluteEncoder = motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        pidController = motor.getPIDController();

        absoluteEncoder.setInverted(true);
        absoluteEncoder.setPositionConversionFactor(2 * Math.PI * FourBarConstants.BELT_RATIO);
        absoluteEncoder.setVelocityConversionFactor(2 * Math.PI * FourBarConstants.BELT_RATIO / 60.0);
        absoluteEncoder.setZeroOffset(FourBarConstants.ELBOW_OFFSET.getRadians());

        pidController.setP(FourBarConstants.ELBOW_P);
        pidController.setI(FourBarConstants.ELBOW_I);
        pidController.setD(FourBarConstants.ELBOW_D);
        pidController.setFeedbackDevice(absoluteEncoder);
        pidController.setPositionPIDWrappingMinInput(0);
        pidController.setPositionPIDWrappingMaxInput(2 * Math.PI * FourBarConstants.BELT_RATIO);
        pidController.setPositionPIDWrappingEnabled(true);

        motor.burnFlash();
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
//        if (lastPos < 0.1 && absoluteEncoder.getPosition() > 0.9) {
//            counter--;
//        }
//        else if (lastPos > 0.9 && absoluteEncoder.getPosition() < 0.1) {
//            counter++;
//        }

        inputs.armPositionRad = absoluteEncoder.getPosition() + counter;
        lastPos = absoluteEncoder.getPosition();
        inputs.armVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(absoluteEncoder.getVelocity());
        inputs.armAppliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.armCurrentAmps = motor.getOutputCurrent();
        inputs.armTempCelsius = motor.getMotorTemperature();
    }

    @Override
    public void setVoltage(double volts) {
//        if (absoluteEncoder.getPosition())
        motor.setVoltage(volts);
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