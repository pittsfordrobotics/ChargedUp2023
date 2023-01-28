package com.team3181.frc2023.subsystems.tank;

import com.team3181.frc2023.Constants.TankConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;

public class TankIOSim implements TankIO {

    private final DifferentialDrivetrainSim drivetrain = new DifferentialDrivetrainSim(
        DCMotor.getNEO(2),
        TankConstants.GEAR_RATIO,
        TankConstants.MOI,
        TankConstants.WEIGHT_KILO,
        TankConstants.WHEEL_DIAMETER_METERS / 2.0,
        TankConstants.TRACK_WIDTH_METERS,
        null
    );

    private double appliedVoltsLeft = 0;
    private double appliedVoltsRight = 0;

    public TankIOSim() {
    }

    @Override
    public void updateInputs(TankIOInputs inputs) {
        drivetrain.update(0.02);
        inputs.leftPositionMeters = drivetrain.getLeftPositionMeters();
        inputs.rightPositionMeters = drivetrain.getRightPositionMeters();
        inputs.leftVelocityMetersPerSec = drivetrain.getLeftVelocityMetersPerSecond();
        inputs.rightVelocityMetersPerSec = drivetrain.getRightVelocityMetersPerSecond();

        inputs.leftAppliedVolts = appliedVoltsLeft;
        inputs.rightAppliedVolts = appliedVoltsRight;

        inputs.leftCurrentAmps = new double[] {drivetrain.getLeftCurrentDrawAmps()};
        inputs.rightCurrentAmps = new double[] {drivetrain.getRightCurrentDrawAmps()};

        inputs.leftTempCelsius = new double[] {0};
        inputs.rightTempCelsius = new double[] {0};

        inputs.gyroConnected = true;
        double lastGyroPosition = inputs.gyroYawPositionRad;
        inputs.gyroYawPositionRad = -drivetrain.getHeading().getRadians();
        inputs.gyroYawVelocityRadPerSec = (inputs.gyroYawPositionRad - lastGyroPosition) / 0.02;
        inputs.gyroPitchPositionRad = 0;
        inputs.gyroRollPositionRad = 0;

    }

    @Override
    public void set(double leftPercent, double rightPercent) {
        setVoltage(MathUtil.clamp(leftPercent,-1, 1) * 12, MathUtil.clamp(rightPercent,-1, 1) * 12);
    }

    @Override
    public void setVoltage(double leftVolts, double rightVolts) {
        appliedVoltsLeft = leftVolts;
        appliedVoltsRight = rightVolts;
        drivetrain.setInputs(leftVolts, rightVolts);
    }

    @Override
    public void resetEncoders() {
        drivetrain.setPose(new Pose2d(0, 0, drivetrain.getHeading()));
    }
}