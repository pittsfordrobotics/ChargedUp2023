package com.team3181.frc2023.subsystems.fourbar;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.team3181.frc2023.Constants.FourBarConstants;
import com.team3181.lib.drivers.LazySparkMax;
import edu.wpi.first.math.util.Units;

public class ArmIOElbowSparkMax implements ArmIO {
    private final LazySparkMax motor;
    private final AbsoluteEncoder absoluteEncoder;
    private double lastPos = FourBarConstants.ELBOW_MATH_OFFSET.getRadians();
    private double wraparoundOffset = 0;
    private double oneEncoderRotation = 2 * Math.PI;

    public ArmIOElbowSparkMax() {
        motor = new LazySparkMax(FourBarConstants.CAN_ELBOW, IdleMode.kBrake, 80, false,false);
        absoluteEncoder = motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

        absoluteEncoder.setInverted(true);
        absoluteEncoder.setPositionConversionFactor(2 * Math.PI);
        absoluteEncoder.setVelocityConversionFactor(2 * Math.PI / 60.0);
        absoluteEncoder.setZeroOffset(FourBarConstants.ELBOW_ABSOLUTE_OFFSET.getRadians());
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);

        motor.burnFlash();
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        double position = absoluteEncoder.getPosition() + FourBarConstants.ELBOW_MATH_OFFSET.getRadians() + wraparoundOffset;
        double positionDiff = position - lastPos;

        // Check if we've wrapped around the zero point.  If we've travelled more than a half circle in one update period,
        // then assume we wrapped around.
        if (positionDiff > oneEncoderRotation / 2) {
            // We went up by over a half rotation, which means we likely wrapped around the zero point going in the negative direction.
            position -= oneEncoderRotation;
            wraparoundOffset -= oneEncoderRotation;
        }
        if (positionDiff < -1 * oneEncoderRotation / 2) {
            // We went down by over a half rotation, which means we likely wrapped around the zero point going in the positive direction.
            position += oneEncoderRotation;
            wraparoundOffset += oneEncoderRotation;
        }

        inputs.armOffsetPositionRad = position;
        inputs.armVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(absoluteEncoder.getVelocity());
        inputs.armAppliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.armCurrentAmps = motor.getOutputCurrent();
        inputs.armTempCelsius = motor.getMotorTemperature();
        lastPos = position;
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
        absoluteEncoder.setZeroOffset(0);
        absoluteEncoder.setZeroOffset(absoluteEncoder.getPosition());
    }
}