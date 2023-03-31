package com.team3181.frc2023.subsystems.vision;

public class VisionIOSim implements VisionIO {
    public VisionIOSim() {}

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.hasTarget = false;
        inputs.connected = true;
        inputs.vAngle = 0;
        inputs.hAngle = 0;
        inputs.tagIDs = new double[]{1, 2, 5, 7};
        inputs.botXYZ = new double[]{4, 4, 0};
        inputs.botRPY = new double[]{0, 0, 1};
    }
}