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

public class ArmIOElbowSparkMax implements ArmIO {
    private final LazySparkMax motor;
    private final SparkMaxLimitSwitch limitSwitch;
    private final AbsoluteEncoder absoluteEncoder;
    private double lastPos = FourBarConstants.ELBOW_MATH_OFFSET.getRadians();
    private double wraparoundOffset = 0;
    private double oneEncoderRotation = 2 * Math.PI;
    private double currentOffset = FourBarConstants.ELBOW_ABSOLUTE_OFFSET.getRadians();

    public ArmIOElbowSparkMax() {
        motor = new LazySparkMax(FourBarConstants.CAN_ELBOW, IdleMode.kBrake, 80, false,false);
        absoluteEncoder = motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        limitSwitch = motor.getReverseLimitSwitch(Type.kNormallyOpen);
        limitSwitch.enableLimitSwitch(true);

        absoluteEncoder.setInverted(true);
        absoluteEncoder.setPositionConversionFactor(2 * Math.PI);
        absoluteEncoder.setVelocityConversionFactor(2 * Math.PI / 60.0);
        absoluteEncoder.setZeroOffset(FourBarConstants.ELBOW_ABSOLUTE_OFFSET.getRadians());
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);

        motor.burnFlash();
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
//        double position = absoluteEncoder.getPosition() + FourBar.mathOffsetElbow.getRadians() + wraparoundOffset;
//        double positionDiff = position - lastPos;
//
////         Check if we've wrapped around the zero point.  If we've travelled more than a half circle in one update period,
////         then assume we wrapped around.
//        if (positionDiff > oneEncoderRotation / 1.2) {
//            // We went up by over a half rotation, which means we likely wrapped around the zero point going in the negative direction.
//            position -= oneEncoderRotation;
//            wraparoundOffset -= oneEncoderRotation;
//        }
//        if (positionDiff < -1 * oneEncoderRotation / 1.22) {
//            // We went down by over a half rotation, which means we likely wrapped around the zero point going in the positive direction.
//            position += oneEncoderRotation;
//            wraparoundOffset += oneEncoderRotation;
//        }

        inputs.armPositionRawRad = absoluteEncoder.getPosition();
        inputs.armOffsetPositionRad = absoluteEncoder.getPosition() + FourBar.mathOffsetElbow.getRadians();
//        inputs.armOffsetPositionRad = position;
        inputs.armVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(absoluteEncoder.getVelocity());
        inputs.armAppliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.armCurrentAmps = motor.getOutputCurrent();
        inputs.armTempCelsius = motor.getMotorTemperature();
        inputs.armAtLimit = limitSwitch.isPressed();
//        lastPos = position;
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void setBrakeMode(boolean enable) {
        motor.setIdleMode(enable ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
    }

    @Override
    public void zeroAbsoluteEncoder() {
        // TODO: update mathoffset with this mess
//        currentOffset = absoluteEncoder.getPosition() - currentOffset + 5;
//        absoluteEncoder.setZeroOffset(currentOffset);
    }

    @Override
    public boolean isAtLimitSwitch() {
        return limitSwitch.isPressed();
    }
}