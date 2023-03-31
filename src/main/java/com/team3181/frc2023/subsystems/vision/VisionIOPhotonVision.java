package com.team3181.frc2023.subsystems.vision;

import com.team3181.lib.util.Alert;
import com.team3181.lib.util.Alert.AlertType;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import java.io.IOException;
import java.util.ArrayList;

public class VisionIOPhotonVision implements VisionIO {
    private AprilTagFieldLayout aprilTagFieldLayout;
    private Alliance lastAlliance = Alliance.Invalid;

    private final PhotonCamera cam;

    // Construct PhotonPoseEstimator
    private PhotonPoseEstimator poseEstimator;

    public VisionIOPhotonVision(String camName, Transform3d transform3d) {
        cam = new PhotonCamera(camName);
        try {
            aprilTagFieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
            poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP, cam, transform3d);
            poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        }
        catch (IOException ignored) {
            new Alert("PhotonVision FAILED to load! Vision WILL be dead!", AlertType.ERROR);
            aprilTagFieldLayout = null;
            poseEstimator = null;
        }
    }

    public void updateInputs(VisionIOInputs inputs) {
        if (DriverStation.getAlliance() != lastAlliance) {
            lastAlliance = DriverStation.getAlliance();
            if (lastAlliance == Alliance.Blue) {
                aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
            }
            else if (lastAlliance == Alliance.Red) {
                aprilTagFieldLayout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
            }
            poseEstimator.setFieldTags(aprilTagFieldLayout);
        }
        if (poseEstimator.update().isPresent()) {
            inputs.captureTimestamp = cam.getLatestResult().getTimestampSeconds();
            inputs.botXYZ = new double[] {poseEstimator.update().get().estimatedPose.getX(), poseEstimator.update().get().estimatedPose.getY(), poseEstimator.update().get().estimatedPose.getZ()};
            inputs.botRPY = new double[] {poseEstimator.update().get().estimatedPose.getRotation().getX(), poseEstimator.update().get().estimatedPose.getRotation().getY(), poseEstimator.update().get().estimatedPose.getRotation().getZ()};
            ArrayList<Integer> tagIDs = new ArrayList<>();
            for (int i = 0; i < cam.getLatestResult().targets.size(); i++) {
                tagIDs.add(cam.getLatestResult().targets.get(i).getFiducialId());
            }
            inputs.tagIDs = tagIDs.stream().mapToDouble(i -> i).toArray();
        }
        else {
            inputs.botXYZ = new double[] {0,0,0};
            inputs.botRPY = new double[] {0,0,0};
        }
        inputs.pipelineLatency = Units.millisecondsToSeconds(cam.getLatestResult().getLatencyMillis());
        inputs.hasTarget = cam.getLatestResult().hasTargets();
        inputs.connected = cam.isConnected();
    }

    @Override
    public void setPipeline(Pipelines pipeline) {
        cam.setPipelineIndex(pipeline.getNum());
    }
}