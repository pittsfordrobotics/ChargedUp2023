package com.team3181.frc2023.subsystems.tank;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.team3181.frc2023.Constants.TankConstants;
import com.team3181.lib.drivers.LazySparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotController;

public class TankIOSparkMax implements TankIO {
    private final LazySparkMax leftPrimary = new LazySparkMax(TankConstants.CAN_LEFT_LEADER, IdleMode.kBrake, 60,false);
    private final LazySparkMax leftFollower = new LazySparkMax(TankConstants.CAN_LEFT_FOLLOWER, IdleMode.kBrake, 60, leftPrimary);
    private final LazySparkMax rightPrimary = new LazySparkMax(TankConstants.CAN_RIGHT_LEADER, IdleMode.kBrake, 60, true);
    private final LazySparkMax rightFollower = new LazySparkMax(TankConstants.CAN_RIGHT_FOLLOWER, IdleMode.kBrake, 60, rightPrimary);

    private final WPI_Pigeon2 pigeon = new WPI_Pigeon2(TankConstants.CAN_PIGEON);
//    private final AHRS navX = new AHRS(SPI.Port.kMXP);

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
        pigeon.calibrate();
        pigeon.reset();
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) {
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
        inputs.gyroYawPositionRad = Math.toRadians(pigeon.getAngle());
        inputs.gyroYawVelocityRadPerSec = Math.toRadians(pigeon.getRate());
        inputs.gyroPitchPositionRad = Math.toRadians(pigeon.getRoll());
        inputs.gyroRollPositionRad = Math.toRadians(pigeon.getPitch());
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