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
import com.team3181.lib.math.BetterMath;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;

public class ArmIOShoulderSparkMax implements ArmIO {
    private final LazySparkMax mainMotor;
    private final LazySparkMax followerMotor;
    private final AbsoluteEncoder absoluteEncoder;
    private final SparkMaxLimitSwitch limitSwitch;
    private Rotation2d offset;
    private int counter = 0;
    private double lastPos = FourBarConstants.SHOULDER_MATH_OFFSET.getRadians();
    private double wraparoundOffset = 0;
    private double oneEncoderRotation = 2 * Math.PI * FourBarConstants.CHAIN_RATIO;

    public ArmIOShoulderSparkMax() {
        mainMotor = new LazySparkMax(FourBarConstants.CAN_SHOULDER_MASTER, IdleMode.kBrake, 80, true, false);
        followerMotor = new LazySparkMax(FourBarConstants.CAN_SHOULDER_FOLLOWER, IdleMode.kBrake, 80, mainMotor, true, true);
        absoluteEncoder = mainMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        limitSwitch = mainMotor.getReverseLimitSwitch(Type.kNormallyOpen);
        limitSwitch.enableLimitSwitch(true);

        absoluteEncoder.setInverted(true);
        absoluteEncoder.setPositionConversionFactor(2 * Math.PI * FourBarConstants.CHAIN_RATIO);
        absoluteEncoder.setVelocityConversionFactor(2 * Math.PI * FourBarConstants.CHAIN_RATIO / 60.0);
        absoluteEncoder.setZeroOffset(FourBarConstants.SHOULDER_ABSOLUTE_OFFSET.getRadians());
        mainMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);

        mainMotor.burnFlash();

        offset = FourBarConstants.SHOULDER_ABSOLUTE_OFFSET;
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        double position = absoluteEncoder.getPosition() + FourBarConstants.SHOULDER_MATH_OFFSET.getRadians() + wraparoundOffset;
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
        
        // if (lastPos < FourBarConstants.SHOULDER_FLIP_MIN.getRadians() + 0.1 && (position) > FourBarConstants.SHOULDER_FLIP_MAX.getRadians() - 0.1 && counter == 1) {
        //     counter--;
        // } else if (lastPos > FourBarConstants.SHOULDER_FLIP_MAX.getRadians() - 0.1 && (position) < FourBarConstants.SHOULDER_FLIP_MIN.getRadians() + 0.1 && counter == 0) {
        //     counter++;
        // }
        //inputs.armOffsetPositionRad = position + counter * (FourBarConstants.SHOULDER_FLIP_MIN.getRadians() * -1 + FourBarConstants.SHOULDER_FLIP_MAX.getRadians());

        inputs.armPositionRawRad = absoluteEncoder.getPosition();
        inputs.armOffsetPositionRad = position;
        inputs.armVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(absoluteEncoder.getVelocity());
        inputs.armAppliedVolts = mainMotor.getAppliedOutput() * mainMotor.getBusVoltage();
        inputs.armCurrentAmps = mainMotor.getOutputCurrent();
        inputs.armTempCelsius = mainMotor.getMotorTemperature();
        inputs.armAtLimit = limitSwitch.isPressed();
        lastPos = position;

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
        absoluteEncoder.setZeroOffset(BetterMath.clampCustom(absoluteEncoder.getPosition() - offset.getRadians() - 0.1, 0,2 * Math.PI * FourBarConstants.CHAIN_RATIO));
    }

    @Override
    public boolean isAtLimitSwitch() {
        return limitSwitch.isPressed();
    }
}