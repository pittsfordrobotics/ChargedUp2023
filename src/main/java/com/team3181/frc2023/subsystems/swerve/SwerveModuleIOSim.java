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
    private final PIDController steerPosPID = new PIDController(15, 0, 0);

    private double steerAbsolutePositionRad = Math.random() * 2.0 * Math.PI;
    private double steerRelativePositionRad = steerAbsolutePositionRad;
    private double driveAppliedVolts = 0.0;
    private double steerAppliedVolts = 0.0;

    public void updateInputs(SwerveModuleIOInputs inputs) {
        driveSim.update(RobotConstants.LOOP_TIME_SECONDS);
        steerSim.update(RobotConstants.LOOP_TIME_SECONDS);

        double angleDiffRad =
                steerSim.getAngularVelocityRadPerSec() * RobotConstants.LOOP_TIME_SECONDS;
        steerRelativePositionRad += angleDiffRad;
        steerAbsolutePositionRad += angleDiffRad;
        while (steerAbsolutePositionRad < 0) {
            steerAbsolutePositionRad += 2.0 * Math.PI;
        }
        while (steerAbsolutePositionRad > 2.0 * Math.PI) {
            steerAbsolutePositionRad -= 2.0 * Math.PI;
        }

        inputs.driveVelocityMetersPerSec = driveSim.getAngularVelocityRadPerSec() * Math.PI * SwerveConstants.WHEEL_DIAMETER_METERS / SwerveConstants.DRIVE_GEAR_RATIO;
        inputs.drivePositionMeters = inputs.drivePositionMeters
                + (inputs.driveVelocityMetersPerSec * RobotConstants.LOOP_TIME_SECONDS);
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());
        inputs.driveTempCelcius = 0;

        inputs.steerAbsolutePositionRad = steerAbsolutePositionRad;
        inputs.steerAbsoluteVelocityRadPerSec = steerSim.getAngularVelocityRadPerSec();
        inputs.steerPositionRad = steerRelativePositionRad;
        inputs.steerVelocityRadPerSec = steerSim.getAngularVelocityRadPerSec();
        inputs.steerAppliedVolts = steerAppliedVolts;
        inputs.steerCurrentAmps = Math.abs(steerSim.getCurrentDrawAmps());
        inputs.steerTempCelcius = 0;
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
        steerPosPID.setSetpoint(state.angle.getRadians());
        steerSim.setInputVoltage(steerPosPID.calculate(steerRelativePositionRad) + (isOpenLoop ? SwerveConstants.MODULE_STEER_FF_OL : SwerveConstants.MODULE_STEER_FF_CL) * state.omegaRadPerSecond);
    }

    @Override
    public void stopMotors() {
        driveSim.setInputVoltage(0);
        steerSim.setInputVoltage(0);
    }
}