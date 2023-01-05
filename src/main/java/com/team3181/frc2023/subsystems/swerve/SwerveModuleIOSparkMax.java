package com.team3181.frc2023.subsystems.swerve;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import com.team3181.frc2023.Constants.SwerveConstants;
import com.team3181.lib.swerve.BetterSwerveModuleState;
import com.team3181.lib.drivers.LazySparkMax;

public class SwerveModuleIOSparkMax implements SwerveModuleIO {
    private final LazySparkMax driveMotor;
    private final LazySparkMax steerMotor;

    private final RelativeEncoder driveRelativeEncoder;
    private final RelativeEncoder steerRelativeEncoder;
    private final RelativeEncoder steerAbsoluteEncoder;

    private final SparkMaxPIDController drivePID;
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(SwerveConstants.MODULE_DRIVE_S, SwerveConstants.MODULE_DRIVE_V, SwerveConstants.MODULE_DRIVE_A);
    private final SparkMaxPIDController steerPID;

    public SwerveModuleIOSparkMax(int driveID, int steerID, Rotation2d offset) {
        driveMotor = new LazySparkMax(driveID, IdleMode.kBrake, 60);
        steerMotor = new LazySparkMax(steerID, IdleMode.kBrake, 40);

        driveRelativeEncoder = driveMotor.getEncoder();
        steerRelativeEncoder = steerMotor.getEncoder();
        steerAbsoluteEncoder = steerMotor.getAlternateEncoder(SwerveConstants.THROUGH_BORE_COUNTS_PER_REV);

        driveRelativeEncoder.setPositionConversionFactor(Math.PI * SwerveConstants.WHEEL_DIAMETER_METERS / SwerveConstants.DRIVE_GEAR_RATIO);
        driveRelativeEncoder.setVelocityConversionFactor(Math.PI * SwerveConstants.WHEEL_DIAMETER_METERS / SwerveConstants.DRIVE_GEAR_RATIO / 60);
        steerRelativeEncoder.setPositionConversionFactor(2 * Math.PI / SwerveConstants.STEER_GEAR_RATIO);
        steerRelativeEncoder.setVelocityConversionFactor(2 * Math.PI / SwerveConstants.STEER_GEAR_RATIO / 60);
        steerAbsoluteEncoder.setPositionConversionFactor(2 * Math.PI);
        steerAbsoluteEncoder.setVelocityConversionFactor(2 * Math.PI / 60);

        drivePID = driveMotor.getPIDController();
        steerPID = steerMotor.getPIDController();

        drivePID.setP(SwerveConstants.MODULE_DRIVE_P);
        drivePID.setI(SwerveConstants.MODULE_DRIVE_I);
        drivePID.setD(SwerveConstants.MODULE_DRIVE_D);

        steerPID.setP(SwerveConstants.MODULE_STEER_P);
        steerPID.setI(SwerveConstants.MODULE_STEER_I);
        steerPID.setD(SwerveConstants.MODULE_STEER_D);

//        saves PID Config
        driveMotor.burnFlash();
        steerMotor.burnFlash();

        steerRelativeEncoder.setPosition(steerAbsoluteEncoder.getPosition() - offset.getRadians());
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.drivePositionMeters = driveRelativeEncoder.getPosition();
        inputs.driveVelocityMetersPerSec = driveRelativeEncoder.getVelocity();
        inputs.driveAppliedVolts = driveMotor.getAppliedVoltage();
        inputs.driveCurrentAmps = driveMotor.getOutputCurrent();
        inputs.driveTempCelcius = driveMotor.getMotorTemperature();

        inputs.steerAbsolutePositionRad = steerAbsoluteEncoder.getPosition();
        inputs.steerAbsoluteVelocityRadPerSec = steerAbsoluteEncoder.getVelocity();
        inputs.steerPositionRad = steerRelativeEncoder.getPosition();
        inputs.steerVelocityRadPerSec = steerRelativeEncoder.getVelocity();
        inputs.steerAppliedVolts = steerMotor.getAppliedVoltage();
        inputs.steerCurrentAmps = steerMotor.getOutputCurrent();
        inputs.steerTempCelcius = steerMotor.getMotorTemperature();
    }

    @Override
    public void setDriveVoltage(double voltage) {
        driveMotor.setVoltage(voltage);
    }

    @Override
    public void setSteerVoltage(double voltage) {
        steerMotor.setVoltage(voltage);
    }

    @Override
    public void setModuleState(BetterSwerveModuleState state, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percent = state.speedMetersPerSecond / SwerveConstants.MAX_LINEAR_VELOCITY_METERS_PER_SECOND;
            driveMotor.set(percent);
        }
        else {
            drivePID.setReference(state.speedMetersPerSecond, ControlType.kVelocity, 0, feedforward.calculate(state.speedMetersPerSecond));
        }
        steerPID.setReference(state.angle.getRadians(), ControlType.kPosition, 0, state.omegaRadPerSecond * (isOpenLoop ? SwerveConstants.MODULE_STEER_FF_OL : SwerveConstants.MODULE_STEER_FF_CL));
    }

    @Override
    public void setDriveBrakeMode(boolean enable) {
        driveMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public void setSteerBrakeMode(boolean enable) {
        steerMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public void stopMotors() {
        driveMotor.stopMotor();
        steerMotor.stopMotor();
    }
}