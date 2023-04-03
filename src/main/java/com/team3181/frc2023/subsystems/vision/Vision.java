package com.team3181.frc2023.subsystems.vision;

import com.team3181.frc2023.Constants.RobotConstants;
import com.team3181.frc2023.Constants.VisionConstants;
import com.team3181.frc2023.FieldConstants;
import com.team3181.frc2023.subsystems.vision.VisionIO.Pipelines;
import com.team3181.lib.util.Alert;
import com.team3181.lib.util.Alert.AlertType;
import com.team3181.lib.util.PoseEstimator.TimestampedVisionUpdate;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import java.util.*;
import java.util.function.Consumer;

public class Vision extends SubsystemBase {
    private final VisionIO[] io;
    private final VisionIOInputsAutoLogged[] inputs = new VisionIOInputsAutoLogged[]{new VisionIOInputsAutoLogged(), new VisionIOInputsAutoLogged(), new VisionIOInputsAutoLogged(), new VisionIOInputsAutoLogged(), new VisionIOInputsAutoLogged()};
    private Consumer<List<TimestampedVisionUpdate>> visionConsumer = (x) -> {};

    private Pipelines pipeline = Pipelines.MID_RANGE;
    private final Map<Integer, Double> lastTagDetectionTimes = new HashMap<>();
    private boolean visionEnabled = true;

    private final Alert limelightAlert = new Alert("Limelight not detected! Vision WILL be hindered!", AlertType.WARNING);
    private final Alert rightSideAlert = new Alert("Right Camera not detected! Vision WILL be hindered!", AlertType.WARNING);
    private final Alert leftSideAlert = new Alert("Left Camera not detected! Vision WILL be hindered!", AlertType.WARNING);
    private final Alert rightFrontAlert = new Alert("Front Right Camera not detected! Vision will NOT work!", AlertType.ERROR);
    private final Alert leftFrontAlert = new Alert("Front Left Camera not detected! Vision will NOT work!", AlertType.ERROR);

    private static final Vision INSTANCE = new Vision(RobotConstants.LIMELIGHT, RobotConstants.PHOTON_LEFT, RobotConstants.PHOTON_RIGHT, RobotConstants.PHOTON_FRONT_LEFT, RobotConstants.PHOTON_FRONT_RIGHT);
    public static Vision getInstance() {
        return INSTANCE;
    }

    // right and left are relative to the robot from birds eye view facing forwards
    private Vision(VisionIO ioLimelight, VisionIO ioPhotonLeft, VisionIO ioPhotonRight, VisionIO ioPhotonFrontLeft, VisionIO ioPhotonFrontRight) {
        io = new VisionIO[]{ioLimelight, ioPhotonLeft, ioPhotonRight, ioPhotonFrontLeft, ioPhotonFrontRight};
        FieldConstants.aprilTags
                .getTags()
                .forEach(
                        (AprilTag tag) -> lastTagDetectionTimes.put(tag.ID, 0.0));
    }

    @Override
    public void periodic() {
        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            io[i].setPipeline(pipeline);
            switch (i) {
                case 0 -> {
                    Logger.getInstance().processInputs("Limelight", inputs[i]);
                    limelightAlert.set(!inputs[i].connected);
                }
                case 1 -> {
                    Logger.getInstance().processInputs("Photon Left", inputs[i]);
                    leftSideAlert.set(!inputs[i].connected);
                }
                case 2 -> {
                    Logger.getInstance().processInputs("Photon Right", inputs[i]);
                    rightSideAlert.set(!inputs[i].connected);
                }
                case 3 -> {
                    Logger.getInstance().processInputs("Photon Front Left", inputs[i]);
                    leftFrontAlert.set(!inputs[i].connected);
                }
                case 4 -> {
                    Logger.getInstance().processInputs("Photon Front Right", inputs[i]);
                    rightFrontAlert.set(!inputs[i].connected);
                }
            }
        }

        Logger.getInstance().recordOutput("Vision/Pipeline", pipeline.toString());

        List<TimestampedVisionUpdate> visionUpdates = new ArrayList<>();

        List<Pose2d> allRobotPoses = new ArrayList<>();
//        Pose estimation
        if (!visionEnabled) {
            for (int i = 0; i < io.length; i++) {
//            exit if data is bad
                if (Arrays.equals(inputs[i].botXYZ, new double[]{0.0, 0.0, 0.0}) || inputs[i].botXYZ.length == 0 || !inputs[i].connected) {
                    continue;
                }
                Pose3d robotPose3d = new Pose3d(inputs[i].botXYZ[0], inputs[i].botXYZ[1], inputs[i].botXYZ[2], new Rotation3d(inputs[i].botRPY[0], inputs[i].botRPY[1], inputs[i].botRPY[2]));

//            exit if off the field
                if (robotPose3d.getX() < -VisionConstants.FIELD_BORDER_MARGIN
                        || robotPose3d.getX() > FieldConstants.fieldLength + VisionConstants.FIELD_BORDER_MARGIN
                        || robotPose3d.getY() < -VisionConstants.FIELD_BORDER_MARGIN
                        || robotPose3d.getY() > FieldConstants.fieldWidth + VisionConstants.FIELD_BORDER_MARGIN
                        || robotPose3d.getZ() < -VisionConstants.Z_MARGIN
                        || robotPose3d.getZ() > VisionConstants.Z_MARGIN) {
                    continue;
                }

                Pose2d robotPose = robotPose3d.toPose2d();
                Logger.getInstance().recordOutput("Vision/Pose" + i, robotPose);

                // Get tag poses and update last detection times
                List<Pose3d> tagPoses = new ArrayList<>();
                for (int z = 0; z < inputs[i].tagIDs.length; z++) {
                    int tagId = (int) inputs[i].tagIDs[z];
                    lastTagDetectionTimes.put(tagId, Timer.getFPGATimestamp());
                    Optional<Pose3d> tagPose = FieldConstants.aprilTags.getTagPose((int) inputs[i].tagIDs[z]);
                    tagPose.ifPresent(tagPoses::add);
                }

                // Calculate average distance to tag
                double totalDistance = 0.0;
                Pose2d[] tagPoses2d = new Pose2d[tagPoses.size()];
                int num = 0;
                for (Pose3d tagPose : tagPoses) {
                    tagPose = FieldConstants.allianceFlipper(tagPose, DriverStation.getAlliance());
                    totalDistance += tagPose.getTranslation().getDistance(robotPose3d.getTranslation());
                    tagPoses2d[num] = tagPose.toPose2d();
                    num++;
                }
                double avgDistance = totalDistance / tagPoses.size();

                // Add to vision updates
                double xyStdDev = VisionConstants.XY_STD_DEV_COEF * Math.pow(avgDistance, 2.0) / tagPoses.size();
                double thetaStdDev = VisionConstants.THETA_STD_DEV_COEF * Math.pow(avgDistance, 2.0) / tagPoses.size();
                Logger.getInstance().recordOutput("Vision/XYstd", xyStdDev);
                Logger.getInstance().recordOutput("Vision/ThetaStd", thetaStdDev);
                visionUpdates.add(
                        new TimestampedVisionUpdate(
                                inputs[i].captureTimestamp, robotPose, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)));


                allRobotPoses.add(robotPose);

                List<Pose3d> allTagPoses = new ArrayList<>();
                for (Map.Entry<Integer, Double> detectionEntry : lastTagDetectionTimes.entrySet()) {
                    if (Timer.getFPGATimestamp() - detectionEntry.getValue() < VisionConstants.TARGET_LOG_SECONDS && FieldConstants.aprilTags.getTagPose(detectionEntry.getKey()).isPresent()) {
                        allTagPoses.add(FieldConstants.aprilTags.getTagPose(detectionEntry.getKey()).get());
                    }
                }
                Logger.getInstance()
                        .recordOutput(
                                "AprilTagVision/TagPoses", allTagPoses.toArray(new Pose3d[0]));

                visionConsumer.accept(visionUpdates);
                Logger.getInstance()
                        .recordOutput(
                                "Vision/TagPoses" + i,
                                tagPoses2d);
            }
        }
        Logger.getInstance().recordOutput("Vision/Poses", allRobotPoses.toArray(new Pose2d[0]));
        Logger.getInstance().recordOutput("Vision/Enabled", visionEnabled);
    }

    public void setDataInterface(Consumer<List<TimestampedVisionUpdate>> visionConsumer) {
        this.visionConsumer = visionConsumer;
    }
    
    public void setVisionEnabled(boolean enabled) {
        visionEnabled = enabled;
    }
    
    public void setPipeline(Pipelines pipeline) {
        this.pipeline = pipeline;
    }
}