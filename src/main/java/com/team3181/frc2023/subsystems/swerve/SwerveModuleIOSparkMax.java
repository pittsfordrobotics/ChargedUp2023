package com.team3181.frc2023.subsystems.swerve;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.team3181.frc2023.Constants.SwerveConstants;
import com.team3181.lib.drivers.LazySparkMax;
import com.team3181.lib.swerve.BetterSwerveModuleState;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleIOSparkMax implements SwerveModuleIO {
    private final LazySparkMax driveMotor;
    private final LazySparkMax steerMotor;

    private final RelativeEncoder driveRelativeEncoder;
    private final AbsoluteEncoder steerAbsoluteEncoder;

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(SwerveConstants.MODULE_DRIVE_S, SwerveConstants.MODULE_DRIVE_V, SwerveConstants.MODULE_DRIVE_A);
    private final SparkMaxPIDController drivePID;
    private final SparkMaxPIDController steerPID;
//    private final PIDTuner driveTuner;
//    private final PIDTuner steerTuner;

    private final Rotation2d steerOffset;

    public SwerveModuleIOSparkMax(int driveID, int steerID, Rotation2d offset) {
        driveMotor = new LazySparkMax(driveID, IdleMode.kBrake, 50, false, false);
        steerMotor = new LazySparkMax(steerID, IdleMode.kBrake, 30, false, false);

        driveRelativeEncoder = driveMotor.getEncoder();
        steerAbsoluteEncoder = steerMotor.getAbsoluteEncoder(Type.kDutyCycle);
        steerAbsoluteEncoder.setInverted(true);

        // allows for faster response time
        driveRelativeEncoder.setAverageDepth(4);
        driveRelativeEncoder.setMeasurementPeriod(20);

        // converts to m/s
        driveRelativeEncoder.setPositionConversionFactor((Math.PI * SwerveConstants.WHEEL_DIAMETER_METERS) / SwerveConstants.DRIVE_GEAR_RATIO);
        driveRelativeEncoder.setVelocityConversionFactor(((Math.PI * SwerveConstants.WHEEL_DIAMETER_METERS) / SwerveConstants.DRIVE_GEAR_RATIO) / 60.0);

        // converts to rad/s
        steerAbsoluteEncoder.setPositionConversionFactor(2.0 * Math.PI);
        steerAbsoluteEncoder.setVelocityConversionFactor(2.0 * Math.PI / 60.0);
        steerAbsoluteEncoder.setZeroOffset(0);

        drivePID = driveMotor.getPIDController();
        steerPID = steerMotor.getPIDController();

        drivePID.setFeedbackDevice(driveRelativeEncoder);
        steerPID.setFeedbackDevice(steerAbsoluteEncoder);
        steerPID.setPositionPIDWrappingEnabled(true);
        steerPID.setPositionPIDWrappingMinInput(0);
        steerPID.setPositionPIDWrappingMaxInput(2 * Math.PI);

        drivePID.setP(SwerveConstants.MODULE_DRIVE_P);
        drivePID.setI(SwerveConstants.MODULE_DRIVE_I);
        drivePID.setD(SwerveConstants.MODULE_DRIVE_D);
        drivePID.setFF(SwerveConstants.MODULE_DRIVE_FF);

        steerPID.setP(SwerveConstants.MODULE_STEER_P);
        steerPID.setI(SwerveConstants.MODULE_STEER_I);
        steerPID.setD(SwerveConstants.MODULE_STEER_D);

//        driveTuner = new PIDTuner("Drive " + driveID, drivePID);
//        steerTuner = new PIDTuner("Steer " + steerID, drivePID);

//        saves PID Config
        driveMotor.burnFlash();
        steerMotor.burnFlash();

        this.steerOffset = offset;
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
//        driveTuner.setPID();
//        steerTuner.setPID();

        inputs.drivePositionMeters = driveRelativeEncoder.getPosition();
        inputs.driveVelocityMetersPerSec = driveRelativeEncoder.getVelocity();
        inputs.driveAppliedVolts = driveMotor.getAppliedOutput() * driveMotor.getBusVoltage();
        inputs.driveCurrentAmps = driveMotor.getOutputCurrent();
        inputs.driveTempCelsius = driveMotor.getMotorTemperature();

        inputs.steerAbsolutePositionRad = steerAbsoluteEncoder.getPosition();
        inputs.steerOffsetAbsolutePositionRad = steerAbsoluteEncoder.getPosition() - steerOffset.getRadians();
        inputs.steerAbsoluteVelocityRadPerSec = steerAbsoluteEncoder.getVelocity();
        inputs.steerAppliedVolts = steerMotor.getAppliedOutput() * driveMotor.getBusVoltage();
        inputs.steerCurrentAmps = steerMotor.getOutputCurrent();
        inputs.steerTempCelsius = steerMotor.getMotorTemperature();
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
//            drivePID.setReference(state.speedMetersPerSecond, ControlType.kVelocity, 0, feedforward.calculate(state.speedMetersPerSecond));
            drivePID.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
        }
        steerPID.setReference(state.angle.plus(steerOffset).getRadians(), ControlType.kPosition, 0, state.omegaRadPerSecond * (isOpenLoop ? SwerveConstants.MODULE_STEER_FF_OL : SwerveConstants.MODULE_STEER_FF_CL));
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