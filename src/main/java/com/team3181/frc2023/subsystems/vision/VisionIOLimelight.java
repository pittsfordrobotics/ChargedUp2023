package com.team3181.frc2023.subsystems.vision;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.littletonrobotics.junction.Logger;

public class VisionIOLimelight implements VisionIO {
    public double captureTimestamp = 0.0;
    public boolean hasTarget = false;
    public boolean connected = false;
    public double heartbeat = 0.0;
    public double vAngle = 0.0;
    public double hAngle = 0.0;
    public double[] botXYZ = new double[]{};
    public double[] botYPR = new double[]{};

    private final NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

    public VisionIOLimelight() {
        setLEDs(LED.OFF);
    }

    @Override
    public synchronized void updateInputs(VisionIOInputs inputs) {
        NetworkTableEntry validEntry = limelight.getEntry("tv");
        NetworkTableEntry latencyEntry = limelight.getEntry("tl");
        NetworkTableEntry hAngleEntry = limelight.getEntry("tx");
        NetworkTableEntry vAngleEntry = limelight.getEntry("ty");
        NetworkTableEntry heartbeatEntry = limelight.getEntry("hb");
        NetworkTableEntry botposeEntry = limelight.getEntry("botpose");

        if (latencyEntry.getLastChange() >= (Logger.getInstance().getRealTimestamp() - 20000)) {
            captureTimestamp = (Logger.getInstance().getRealTimestamp() / 1000000.0) - Units.millisecondsToSeconds(latencyEntry.getDouble(0.0));
            hasTarget = validEntry.getDouble(0.0) == 1.0;
            connected = heartbeat > heartbeatEntry.getDouble(0.0) && heartbeatEntry.getDouble(0.0) != 0;
            heartbeat = heartbeatEntry.getDouble(0.0);
            vAngle = vAngleEntry.getDouble(0.0);
            hAngle = hAngleEntry.getDouble(0.0);
            botXYZ = new double[]{botposeEntry.getDoubleArray(new double[]{})[0], botposeEntry.getDoubleArray(new double[]{})[1], botposeEntry.getDoubleArray(new double[]{})[2]};
            botYPR = new double[]{botposeEntry.getDoubleArray(new double[]{})[3], botposeEntry.getDoubleArray(new double[]{})[4], botposeEntry.getDoubleArray(new double[]{})[5]};
        }
        inputs.captureTimestamp = captureTimestamp;
        inputs.hasTarget = hasTarget;
        inputs.connected = connected;
        inputs.vAngle = vAngle;
        inputs.hAngle = hAngle;
        inputs.botXYZ = botXYZ;
        inputs.botYPR = botYPR;
    }

    @Override
    public void setPipeline(Pipelines pipeline) {
        limelight.getEntry("pipeline").setDouble(pipeline.getNum());
    }

    @Override
    public void setCameraModes(CameraMode camera) {
        limelight.getEntry("camMode").setDouble(camera.getNum());
    }

    @Override
    public void setLEDs(LED led) {
        limelight.getEntry("ledMode").setDouble(led.getNum());
    }
}