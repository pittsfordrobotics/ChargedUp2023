package com.team3181.frc2023.subsystems.swerve;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.team3181.frc2023.Constants.SwerveConstants;

public class GyroIOPigeon implements GyroIO{
    private final WPI_Pigeon2 pigeon = new WPI_Pigeon2(SwerveConstants.CAN_PIGEON);

    public GyroIOPigeon() {
        pigeon.configAllSettings(SwerveConstants.PIGEON_CONFIG);
        pigeon.reset();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = pigeon.getUpTime() > 0;
        inputs.yawPositionRad = Math.toRadians(pigeon.getAngle());
        inputs.yawVelocityRadPerSec = Math.toRadians(pigeon.getRate());
        inputs.pitchPositionRad = Math.toRadians(pigeon.getRoll());
        inputs.rollPositionRad = Math.toRadians(pigeon.getPitch());
    }
}