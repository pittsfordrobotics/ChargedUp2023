package com.team3181.frc2023.subsystems.endeffector;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.team3181.frc2023.Constants;
import com.team3181.lib.drivers.LazySparkMax;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;

public class EndEffectorIOSparkMax implements EndEffectorIO {
    private final LazySparkMax motor = new LazySparkMax(Constants.EndEffectorConstants.INTAKE_CAN_MAIN, CANSparkMax.IdleMode.kBrake, 30); // currentlimit stolen from 2022 intake
    private final RelativeEncoder encoder = motor.getEncoder();

    public EndEffectorIOSparkMax() {}

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity()) / Constants.EndEffectorConstants.GEARING;
        inputs.appliedVolts = motor.getAppliedOutput() * RobotController.getBatteryVoltage();
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
