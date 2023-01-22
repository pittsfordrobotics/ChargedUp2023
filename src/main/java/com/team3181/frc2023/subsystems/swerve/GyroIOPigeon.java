package com.team3181.frc2023.subsystems.swerve;

import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.team3181.frc2023.Constants.SwerveConstants;

public class GyroIOPigeon implements GyroIO{
    private final WPI_Pigeon2 pigeon = new WPI_Pigeon2(SwerveConstants.CAN_PIGEON);
    private final double[] xyz = new double[3];
    private final double[] ypr = new double[3];

    public GyroIOPigeon() {
        pigeon.configAllSettings(SwerveConstants.PIGEON_CONFIG);
        pigeon.zeroGyroBiasNow();
        pigeon.reset();
        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_1_General, 255);
        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, 20);
        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_3_GeneralAccel, 255);
        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_6_SensorFusion, 255);
        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_10_SixDeg_Quat, 255);
        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_2_Gyro, 255);
        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_4_Mag, 255);
        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_6_Accel, 255);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = pigeon.getUpTime() > 0;
        pigeon.getRawGyro(xyz);
        pigeon.getYawPitchRoll(ypr);
        inputs.yawPositionRad = Math.toRadians(pigeon.getAngle());
        inputs.yawVelocityRadPerSec = Math.toRadians(pigeon.getRate());
        inputs.pitchPositionRad = Math.toRadians(pigeon.getPitch());
        inputs.rollPositionRad = Math.toRadians(pigeon.getRoll());
    }
}