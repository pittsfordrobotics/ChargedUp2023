package com.team3181.frc2023.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import com.team3181.frc2023.Constants.RobotConstants;
import com.team3181.frc2023.Constants.SwerveConstants;
import com.team3181.lib.swerve.BetterSwerveModuleState;

public class SwerveModuleIOSim implements SwerveModuleIO {
    private final FlywheelSim driveSim = new FlywheelSim(DCMotor.getNEO(1), SwerveConstants.DRIVE_GEAR_RATIO, 0.025);
    private final FlywheelSim steerSim = new FlywheelSim(DCMotor.getNeo550(1), SwerveConstants.STEER_GEAR_RATIO, 0.004096955);
    private final PIDController drivePID = new PIDController(10, 0, 0);
    private final PIDController steerPID = new PIDController(15, 0, 0);

    private double steerOffsetAbsolutePositionRad = Math.random() * 2.0 * Math.PI;
    private double steerRelativePositionRad = steerOffsetAbsolutePositionRad;
    private double driveAppliedVolts = 0.0;
    private double steerAppliedVolts = 0.0;

    public SwerveModuleIOSim() {
        steerPID.enableContinuousInput(0, 2 * Math.PI);
    }

    public void updateInputs(SwerveModuleIOInputs inputs) {
        driveSim.update(RobotConstants.LOOP_TIME_SECONDS);
        steerSim.update(RobotConstants.LOOP_TIME_SECONDS);

        double angleDiffRad = steerSim.getAngularVelocityRadPerSec() * RobotConstants.LOOP_TIME_SECONDS;
        steerRelativePositionRad += angleDiffRad;
        steerOffsetAbsolutePositionRad += angleDiffRad;
        while (steerOffsetAbsolutePositionRad < 0) {
            steerOffsetAbsolutePositionRad += 2.0 * Math.PI;
        }
        while (steerOffsetAbsolutePositionRad > 2.0 * Math.PI) {
            steerOffsetAbsolutePositionRad -= 2.0 * Math.PI;
        }

        inputs.driveVelocityMetersPerSec = driveSim.getAngularVelocityRadPerSec() * Math.PI * SwerveConstants.WHEEL_DIAMETER_METERS / SwerveConstants.DRIVE_GEAR_RATIO;
        inputs.drivePositionMeters = inputs.drivePositionMeters
                + (inputs.driveVelocityMetersPerSec * RobotConstants.LOOP_TIME_SECONDS);
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());
        inputs.driveTempCelsius = 0;

        inputs.steerAbsolutePositionRad = steerOffsetAbsolutePositionRad;
        inputs.steerOffsetAbsolutePositionRad = steerOffsetAbsolutePositionRad;
        inputs.steerAbsoluteVelocityRadPerSec = steerSim.getAngularVelocityRadPerSec();
        inputs.steerAppliedVolts = steerAppliedVolts;
        inputs.steerCurrentAmps = Math.abs(steerSim.getCurrentDrawAmps());
        inputs.steerTempCelsius = 0;
    }

    @Override
    public void setDriveVoltage(double voltage) {
        driveAppliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
        driveSim.setInputVoltage(driveAppliedVolts);
    }

    @Override
    public void setSteerVoltage(double voltage) {
        steerAppliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
        steerSim.setInputVoltage(steerAppliedVolts);
    }

    @Override
    public void setModuleState(BetterSwerveModuleState state, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percent = state.speedMetersPerSecond / SwerveConstants.MAX_LINEAR_VELOCITY_METERS_PER_SECOND;
            driveSim.setInputVoltage(percent * 12);
        }
        else {
            drivePID.setSetpoint(state.speedMetersPerSecond);
            driveSim.setInputVoltage(drivePID.calculate(driveSim.getAngularVelocityRadPerSec() * Math.PI * SwerveConstants.WHEEL_DIAMETER_METERS / SwerveConstants.DRIVE_GEAR_RATIO));
        }
        steerPID.setSetpoint(state.angle.getRadians());
        steerSim.setInputVoltage(steerPID.calculate(steerOffsetAbsolutePositionRad) + (isOpenLoop ? SwerveConstants.MODULE_STEER_FF_OL : SwerveConstants.MODULE_STEER_FF_CL) * state.omegaRadPerSecond);
    }

    @Override
    public void stopMotors() {
        driveSim.setInputVoltage(0);
        steerSim.setInputVoltage(0);
    }
}