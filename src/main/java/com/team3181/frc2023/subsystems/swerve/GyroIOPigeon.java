package com.team3181.frc2023.subsystems.swerve;

import com.ctre.phoenix.sensors.Pigeon2;
import com.team3181.frc2023.Constants.SwerveConstants;
import edu.wpi.first.math.util.Units;

public class GyroIOPigeon implements GyroIO {
    private final Pigeon2 pigeon = new Pigeon2(SwerveConstants.CAN_PIGEON);
    private final double[] xyz = new double[3];
    private final double[] ypr = new double[3];

    public GyroIOPigeon() {
        pigeon.configAllSettings(SwerveConstants.PIGEON_CONFIG);
        pigeon.zeroGyroBiasNow();
        pigeon.setYaw(0);
//        can try these out if CAN bus is overloaded
//        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_1_General, 255);
//        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_2_GeneralCompass, 255);
//        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_3_GeneralAccel, 255);
//        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_6_SensorFusion, 255);
//        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, 20);
//        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_10_SixDeg_Quat, 255);
//        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_11_GyroAccum, 255);
//        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.RawStatus_4_Mag, 255);
//        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_2_Gyro, 255);
//        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_4_Mag, 255);
//        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_6_Accel, 255);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = pigeon.getUpTime() > 0;
        pigeon.getRawGyro(xyz);
        pigeon.getYawPitchRoll(ypr);
        inputs.yawPositionRad = Units.degreesToRadians(-ypr[0]); // cw+
        inputs.pitchPositionRad = Units.degreesToRadians(-ypr[1]); // up+ down-
        inputs.rollPositionRad = Units.degreesToRadians(ypr[2]); // cw+
        inputs.rollVelocityRadPerSec = Units.degreesToRadians(xyz[0]); // cw+
        inputs.pitchVelocityRadPerSec = Units.degreesToRadians(-xyz[1]); // up+ down-
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(-xyz[2]); // cw+
    }
}