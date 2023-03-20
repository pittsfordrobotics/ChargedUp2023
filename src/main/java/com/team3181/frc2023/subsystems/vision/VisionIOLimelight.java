package com.team3181.frc2023.subsystems.vision;
import com.team3181.lib.util.LimelightHelpers;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.littletonrobotics.junction.Logger;

public class VisionIOLimelight implements VisionIO {
    public Pose3d lastPose = new Pose3d();

    private final String limelightName = "";
    private final NetworkTable limelight = LimelightHelpers.getLimelightNTTable(limelightName);

    public VisionIOLimelight() {
        setLEDs(LED.OFF);
    }

    public void updateInputs(VisionIOInputs inputs) {
        NetworkTableEntry heartbeatEntry = limelight.getEntry("hb");
        NetworkTableEntry botposeEntry = DriverStation.getAlliance() == Alliance.Blue ? limelight.getEntry("botpose_wpiblue") : limelight.getEntry("botpose_wpired");

        double pipelineLatency = LimelightHelpers.getLatency_Pipeline(limelightName);
        double captureLatency = LimelightHelpers.getLatency_Capture(limelightName);
        double totalLatency = pipelineLatency + captureLatency; // ms

        if (!lastPose.equals(new Pose3d(botposeEntry.getDoubleArray(new double[7])[0], botposeEntry.getDoubleArray(new double[7])[1], botposeEntry.getDoubleArray(new double[7])[2], new Rotation3d(botposeEntry.getDoubleArray(new double[7])[3], botposeEntry.getDoubleArray(new double[7])[4], botposeEntry.getDoubleArray(new double[7])[5])))) {
            inputs.captureTimestamp = (Logger.getInstance().getRealTimestamp() / 1000000.0) - Units.millisecondsToSeconds(totalLatency);
            inputs.botXYZ = new double[]{botposeEntry.getDoubleArray(new double[7])[0], botposeEntry.getDoubleArray(new double[7])[1], botposeEntry.getDoubleArray(new double[7])[2]};
            inputs.botRPY = new double[]{botposeEntry.getDoubleArray(new double[7])[3], botposeEntry.getDoubleArray(new double[7])[4], botposeEntry.getDoubleArray(new double[7])[5]};
            lastPose = new Pose3d(botposeEntry.getDoubleArray(new double[7])[0], botposeEntry.getDoubleArray(new double[7])[1], botposeEntry.getDoubleArray(new double[7])[2], new Rotation3d(botposeEntry.getDoubleArray(new double[7])[3], botposeEntry.getDoubleArray(new double[7])[4], botposeEntry.getDoubleArray(new double[7])[5]));
        }
        inputs.captureLatency = captureLatency;
        inputs.pipelineLatency = pipelineLatency;
        inputs.hasTarget = LimelightHelpers.getTV(limelightName);
        inputs.connected = heartbeatEntry.getDouble(0.0) > 0.0;
        inputs.vAngle = LimelightHelpers.getTY(limelightName);
        inputs.hAngle = LimelightHelpers.getTX(limelightName);
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