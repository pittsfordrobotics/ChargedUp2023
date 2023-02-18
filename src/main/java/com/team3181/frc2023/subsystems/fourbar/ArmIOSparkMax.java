package com.team3181.frc2023.subsystems.fourbar;

import com.revrobotics.CANSparkMax;
import com.team3181.frc2023.Constants;
import com.team3181.lib.drivers.LazySparkMax;

public class FourBarIOSparkMax implements ArmIO {
    public final LazySparkMax shoulder = new LazySparkMax(Constants.FourBarConstants.CAN_SHOULDER, CANSparkMax.IdleMode.kBrake, 30);
    public final LazySparkMax elbow = new LazySparkMax(Constants.FourBarConstants.CAN_ELBOW, CANSparkMax.IdleMode.kBrake, 30);

    public ArmIOSparkMax() {};

    @Override
    default void updateInputs(ArmIOInputs inputs) {
        inputs.velocityRadPerSec
    }

    @Override
    default void setVoltage(double volts) {}

    @Override
    default void setBrakeMode(boolean enable) {}
}
