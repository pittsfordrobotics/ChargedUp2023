package com.team3181.frc2023.subsystems.fourbar;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import com.team3181.frc2023.Constants.FourBarConstants;
import com.team3181.lib.drivers.LazySparkMax;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;

public class ArmIOShoulderSparkMax implements ArmIO {
    private final LazySparkMax mainMotor;
    private final LazySparkMax followerMotor;
    private final AbsoluteEncoder absoluteEncoder;
    private final SparkMaxLimitSwitch limitSwitch;
    private int counter = 0;
    private double lastPos = 0;

    public ArmIOShoulderSparkMax() {
        mainMotor = new LazySparkMax(FourBarConstants.CAN_SHOULDER_MASTER, IdleMode.kCoast, 80, true, false);
        followerMotor = new LazySparkMax(FourBarConstants.CAN_SHOULDER_FOLLOWER, IdleMode.kCoast, 80, mainMotor, true, true);
        absoluteEncoder = mainMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        limitSwitch = mainMotor.getReverseLimitSwitch(Type.kNormallyOpen);
        limitSwitch.enableLimitSwitch(true);

        absoluteEncoder.setInverted(true);
        absoluteEncoder.setPositionConversionFactor(2 * Math.PI * FourBarConstants.CHAIN_RATIO);
        absoluteEncoder.setVelocityConversionFactor(2 * Math.PI * FourBarConstants.CHAIN_RATIO / 60.0);
        absoluteEncoder.setZeroOffset(FourBarConstants.SHOULDER_ABSOLUTE_OFFSET.getRadians());
        mainMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);

        mainMotor.burnFlash();
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        if (lastPos < FourBarConstants.SHOULDER_FLIP_MIN.getRadians() + 0.1 && (absoluteEncoder.getPosition() + FourBarConstants.SHOULDER_MATH_OFFSET.getRadians()) > FourBarConstants.SHOULDER_FLIP_MAX.getRadians() - 0.1 && counter == 1) {
            counter--;
        } else if (lastPos > FourBarConstants.SHOULDER_FLIP_MAX.getRadians() - 0.1 && (absoluteEncoder.getPosition() + FourBarConstants.SHOULDER_MATH_OFFSET.getRadians()) < FourBarConstants.SHOULDER_FLIP_MIN.getRadians() + 0.1 && counter == 0) {
            counter++;
        }

        inputs.armOffsetPositionRad = absoluteEncoder.getPosition() + FourBarConstants.SHOULDER_MATH_OFFSET.getRadians() + counter * (FourBarConstants.SHOULDER_FLIP_MIN.getRadians() * -1 + FourBarConstants.SHOULDER_FLIP_MAX.getRadians());
        inputs.armVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(absoluteEncoder.getVelocity());
        lastPos = absoluteEncoder.getPosition() + FourBarConstants.SHOULDER_MATH_OFFSET.getRadians();
        inputs.armAppliedVolts = mainMotor.getAppliedOutput() * mainMotor.getBusVoltage();
        inputs.armCurrentAmps = mainMotor.getOutputCurrent();
        inputs.armTempCelsius = mainMotor.getMotorTemperature();
        inputs.armAtLimit = limitSwitch.isPressed();

        Logger.getInstance().recordOutput("Shoulder/Counter", counter);

        Logger.getInstance().recordOutput("Shoulder/followerArmAppliedVolts", followerMotor.getAppliedOutput() * followerMotor.getBusVoltage());
        Logger.getInstance().recordOutput("Shoulder/followerArmCurrentAmps", followerMotor.getOutputCurrent());
        Logger.getInstance().recordOutput("Shoulder/followerArmTempCelsius", followerMotor.getMotorTemperature());
    }

    @Override
    public void setVoltage(double volts) {
        mainMotor.setVoltage(volts);
    }

    @Override
    public void setBrakeMode(boolean enable) {
        mainMotor.setIdleMode(enable ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
    }

    @Override
    public void zeroAbsoluteEncoder() {
        absoluteEncoder.setZeroOffset(absoluteEncoder.getPosition() - FourBarConstants.SHOULDER_ABSOLUTE_OFFSET.getRadians() - 0.1);
    }

    @Override
    public boolean isAtLimitSwitch() {
        return limitSwitch.isPressed();
    }
}