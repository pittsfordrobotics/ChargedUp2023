package com.team3181.frc2023.subsystems.fourbar;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.team3181.frc2023.Constants.FourBarConstants;
import com.team3181.lib.drivers.LazySparkMax;
import edu.wpi.first.math.util.Units;

public class ArmIOShoulderSparkMax implements ArmIO {
    private final LazySparkMax mainMotor;
    private final LazySparkMax followerMotor;
    private final AbsoluteEncoder absoluteEncoder;
    private final SparkMaxPIDController pidController;
    private double lastPos = 0;
    private double counter = 0;

    public ArmIOShoulderSparkMax() {
        mainMotor = new LazySparkMax(FourBarConstants.CAN_SHOULDER_MASTER, IdleMode.kBrake, 80, false, false);
        followerMotor = new LazySparkMax(FourBarConstants.CAN_SHOULDER_FOLLOWER, IdleMode.kBrake, 80, mainMotor, false, true);
        absoluteEncoder = mainMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        pidController = mainMotor.getPIDController();

        absoluteEncoder.setInverted(true);
        absoluteEncoder.setPositionConversionFactor(2 * Math.PI * FourBarConstants.BELT_RATIO);
        absoluteEncoder.setVelocityConversionFactor(2 * Math.PI * FourBarConstants.BELT_RATIO / 60.0);
        absoluteEncoder.setZeroOffset(FourBarConstants.SHOULDER_OFFSET.getRadians());

        pidController.setP(FourBarConstants.SHOULDER_P);
        pidController.setI(FourBarConstants.SHOULDER_I);
        pidController.setD(FourBarConstants.SHOULDER_D);
        pidController.setFeedbackDevice(absoluteEncoder);
        pidController.setPositionPIDWrappingMinInput(0);
        pidController.setPositionPIDWrappingMaxInput(2 * Math.PI * FourBarConstants.BELT_RATIO);
        pidController.setPositionPIDWrappingEnabled(true);

        mainMotor.burnFlash();
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
        inputs.armAppliedVolts = mainMotor.getAppliedOutput() * mainMotor.getBusVoltage();
        inputs.armCurrentAmps = mainMotor.getOutputCurrent();
        inputs.armTempCelsius = mainMotor.getMotorTemperature();
    }

    @Override
    public void setVoltage(double volts) {
        mainMotor.setVoltage(volts);
    }

    @Override
    public void setBrakeMode(boolean enable) {
        mainMotor.setIdleMode(enable ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
    }
}