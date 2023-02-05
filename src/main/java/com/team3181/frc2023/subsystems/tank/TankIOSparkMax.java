package com.team3181.frc2023.subsystems.tank;

import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.team3181.frc2023.Constants.SwerveConstants;
import com.team3181.frc2023.Constants.TankConstants;
import com.team3181.lib.drivers.LazySparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;

public class TankIOSparkMax implements TankIO {
    private final LazySparkMax leftPrimary = new LazySparkMax(TankConstants.CAN_LEFT_LEADER, IdleMode.kBrake, 60,false, true);
    private final LazySparkMax leftFollower = new LazySparkMax(TankConstants.CAN_LEFT_FOLLOWER, IdleMode.kBrake, 60, leftPrimary);
    private final LazySparkMax rightPrimary = new LazySparkMax(TankConstants.CAN_RIGHT_LEADER, IdleMode.kBrake, 60, true, true);
    private final LazySparkMax rightFollower = new LazySparkMax(TankConstants.CAN_RIGHT_FOLLOWER, IdleMode.kBrake, 60, rightPrimary);

    private final Pigeon2 pigeon = new Pigeon2(SwerveConstants.CAN_PIGEON);
    private final double[] xyz = new double[3];
    private final double[] ypr = new double[3];

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    public TankIOSparkMax() {
        leftEncoder = leftPrimary.getEncoder();
        rightEncoder = rightPrimary.getEncoder();

        leftEncoder.setPositionConversionFactor(Math.PI * TankConstants.WHEEL_DIAMETER_METERS / TankConstants.GEAR_RATIO);
        rightEncoder.setPositionConversionFactor(Math.PI * TankConstants.WHEEL_DIAMETER_METERS / TankConstants.GEAR_RATIO);
        leftEncoder.setVelocityConversionFactor(Math.PI * TankConstants.WHEEL_DIAMETER_METERS / TankConstants.GEAR_RATIO / 60);
        rightEncoder.setVelocityConversionFactor(Math.PI * TankConstants.WHEEL_DIAMETER_METERS / TankConstants.GEAR_RATIO / 60);

        pigeon.configAllSettings(TankConstants.PIGEON_CONFIG);
        pigeon.zeroGyroBiasNow();
        pigeon.setYaw(0);
    }

    @Override
    public void updateInputs(TankIOInputs inputs) {
        inputs.leftPositionMeters = leftEncoder.getPosition();
        inputs.rightPositionMeters = rightEncoder.getPosition();
        inputs.leftVelocityMetersPerSec = leftEncoder.getVelocity();
        inputs.rightVelocityMetersPerSec = rightEncoder.getVelocity();

        inputs.leftAppliedVolts =
                leftPrimary.getAppliedOutput() * RobotController.getBatteryVoltage();
        inputs.rightAppliedVolts =
                rightPrimary.getAppliedOutput() * RobotController.getBatteryVoltage();

        inputs.leftCurrentAmps = new double[] {leftPrimary.getOutputCurrent(),
                leftFollower.getOutputCurrent()};
        inputs.rightCurrentAmps = new double[] {rightPrimary.getOutputCurrent(),
                rightFollower.getOutputCurrent()};

        inputs.leftTempCelsius = new double[] {leftPrimary.getMotorTemperature(),
                leftFollower.getMotorTemperature()};
        inputs.rightTempCelsius = new double[] {rightPrimary.getMotorTemperature(),
                rightFollower.getMotorTemperature()};

        inputs.gyroConnected = pigeon.getUpTime() > 0;
        pigeon.getRawGyro(xyz);
        pigeon.getYawPitchRoll(ypr);
        inputs.gyroYawPositionRad = Units.degreesToRadians(ypr[0]); // ccw+
        inputs.gyroPitchPositionRad = Units.degreesToRadians(-ypr[1]); // up+ down-
        inputs.gyroRollPositionRad = Units.degreesToRadians(ypr[2]); // cw+
        inputs.gyroRollVelocityRadPerSec = Units.degreesToRadians(xyz[0]); // cw+
        inputs.gyroPitchVelocityRadPerSec = Units.degreesToRadians(-xyz[1]); // up+ down-
        inputs.gyroYawVelocityRadPerSec = Units.degreesToRadians(xyz[2]); // ccw+
    }

    @Override
    public void set(double leftPercent, double rightPercent) {
        setVoltage(MathUtil.clamp(leftPercent,-1, 1) * 12, MathUtil.clamp(rightPercent,-1, 1) * 12);
    }

    @Override
    public void setVoltage(double leftVolts, double rightVolts) {
        leftPrimary.setVoltage(leftVolts);
        rightPrimary.setVoltage(rightVolts);
    }

    @Override
    public void setBrakeMode(boolean enable) {
        IdleMode mode = enable ? IdleMode.kBrake : IdleMode.kCoast;
        leftPrimary.setIdleMode(mode);
        leftFollower.setIdleMode(mode);
        rightPrimary.setIdleMode(mode);
        rightFollower.setIdleMode(mode);
    }

    @Override
    public void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }
}