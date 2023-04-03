package com.team3181.frc2023.subsystems.endeffector;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.team3181.frc2023.Constants;
import com.team3181.lib.drivers.LazySparkMax;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;

public class EndEffectorIOSparkMax implements EndEffectorIO {
    private final LazySparkMax motorLeft = new LazySparkMax(Constants.EndEffectorConstants.CAN_LEFT, IdleMode.kCoast, 30, false, false);
    private final LazySparkMax motorRight = new LazySparkMax(Constants.EndEffectorConstants.CAN_RIGHT, IdleMode.kCoast, 30, motorLeft, true, false);
    private final RelativeEncoder encoder = motorLeft.getEncoder();

    public EndEffectorIOSparkMax() {
        encoder.setPositionConversionFactor(Constants.EndEffectorConstants.GEARING);
        encoder.setVelocityConversionFactor(Constants.EndEffectorConstants.GEARING / 60.0);
        motorLeft.burnFlash();
    }

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        // motor left is master
        inputs.positionRad = encoder.getPosition();
        inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity());
        inputs.appliedVolts = motorLeft.getAppliedOutput() * motorLeft.getBusVoltage();
        inputs.currentAmps = motorLeft.getOutputCurrent();
        inputs.tempCelsius = motorLeft.getMotorTemperature();

        Logger.getInstance().recordOutput("EndEffector/rightPositionRad", motorRight.getEncoder().getPosition());
        Logger.getInstance().recordOutput("EndEffector/rightVelocityRadPerSec", Units.rotationsPerMinuteToRadiansPerSecond(motorRight.getEncoder().getVelocity()));
        Logger.getInstance().recordOutput("EndEffector/rightAppliedVolts", motorRight.getAppliedOutput() * motorRight.getBusVoltage());
        Logger.getInstance().recordOutput("EndEffector/rightCurrentAmps", motorRight.getOutputCurrent());
        Logger.getInstance().recordOutput("EndEffector/rightTempCelsius", motorRight.getMotorTemperature());
    }

    @Override
    public void setVoltage(double volts) {
        motorLeft.setVoltage(volts);
    }

    @Override
    public void setBrakeMode(boolean enable) {
        motorLeft.setIdleMode(enable ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
    }
}