package com.team3181.frc2023.subsystems.vision;

import com.team3181.frc2023.Constants.RobotConstants;
import com.team3181.frc2023.Constants.VisionConstants;
import com.team3181.frc2023.FieldConstants;
import com.team3181.frc2023.subsystems.vision.VisionIO.CameraMode;
import com.team3181.frc2023.subsystems.vision.VisionIO.Pipelines;
import com.team3181.lib.math.GeomUtil;
import com.team3181.lib.util.Alert;
import com.team3181.lib.util.Alert.AlertType;
import com.team3181.lib.util.PoseEstimator.TimestampedVisionUpdate;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import java.util.*;
import java.util.function.Consumer;

public class Vision extends SubsystemBase {
    private final VisionIO ioLimelight;
    private final VisionIO ioPhotonLeft;
    private final VisionIO ioPhotonRight;
    private final VisionIO ioPhotonFrontLeft;
    private final VisionIO ioPhotonFrontRight;
    private final VisionIOInputsAutoLogged inputsLimelight = new VisionIOInputsAutoLogged();
    private final VisionIOInputsAutoLogged inputsPhotonFrontLeft = new VisionIOInputsAutoLogged();
    private final VisionIOInputsAutoLogged inputsPhotonFrontRight = new VisionIOInputsAutoLogged();
    private final VisionIOInputsAutoLogged inputsPhotonRight = new VisionIOInputsAutoLogged();
    private final VisionIOInputsAutoLogged inputsPhotonLeft = new VisionIOInputsAutoLogged();
    private final VisionIO[] io;
    private final VisionIOInputsAutoLogged[] inputs;
    private Consumer<List<TimestampedVisionUpdate>> visionConsumer = (x) -> {};

    private Pipelines pipeline = Pipelines.MID_RANGE;
//    private LED led = LED.PIPELINE;
    private CameraMode camera = CameraMode.VISION_PROCESSING;
    private final Timer timer = new Timer();
    private boolean timerStarted = false;
    private boolean enabled = false;
    private Pose2d lastPose = new Pose2d();
    private Map<Integer, Double> lastTagDetectionTimes = new HashMap<>();

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
        this.ioLimelight = ioLimelight;
        this.ioPhotonLeft = ioPhotonLeft;
        this.ioPhotonRight = ioPhotonRight;
        this.ioPhotonFrontLeft = ioPhotonFrontLeft;
        this.ioPhotonFrontRight = ioPhotonFrontRight;
        io = new VisionIO[]{ioLimelight, ioPhotonLeft, ioPhotonRight, ioPhotonFrontLeft, ioPhotonFrontRight};
        inputs = new VisionIOInputsAutoLogged[]{inputsLimelight, inputsPhotonLeft, inputsPhotonRight, inputsPhotonFrontLeft, inputsPhotonFrontRight};
    }

    @Override
    public void periodic() {
        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            io[i].setPipeline(pipeline);
            io[i].setCameraModes(camera);
        }
        Logger.getInstance().processInputs("Limelight", inputsLimelight);
        Logger.getInstance().processInputs("Photon Left", inputsPhotonLeft);
        Logger.getInstance().processInputs("Photon Right", inputsPhotonRight);
        Logger.getInstance().processInputs("Photon Front Left", inputsPhotonFrontLeft);
        Logger.getInstance().processInputs("Photon Front Right", inputsPhotonFrontRight);

        limelightAlert.set(!inputsLimelight.connected);
//        rightSideAlert.set(!inputsPhotonRight.connected);
//        leftSideAlert.set(!inputsPhotonLeft.connected);
//        rightFrontAlert.set(!inputsPhotonFrontRight.connected);
//        leftFrontAlert.set(!inputsPhotonFrontLeft.connected);

        Logger.getInstance().recordOutput("Vision/Pipeline", pipeline.toString());
        Logger.getInstance().recordOutput("Vision/Camera", camera.toString());
        Logger.getInstance().recordOutput("Vision/Enabled", enabled);

        List<TimestampedVisionUpdate> visionUpdates = new ArrayList<>();

        List<Pose2d> allRobotPoses = new ArrayList<>();
//        Pose estimation
        for (int i = 0; i < io.length; i++) {
//            exit if data is bad
            if (Arrays.equals(inputs[i].botXYZ, new double[]{0, 0, 0}) || inputs[i].botXYZ.length == 0 || !inputs[i].connected) {
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
            visionConsumer.accept(visionUpdates);
            Logger.getInstance()
                    .recordOutput(
                            "Vision/TagPoses" + i,
                            tagPoses2d);
        }
        Logger.getInstance().recordOutput("Vision/Poses", allRobotPoses.toArray(new Pose2d[0]));
//        if (getPose() != null) {
//            if ((!DriverStation.isAutonomous() && (!Objects.equals(getPose(), new Pose2d()) && inputsLimelight.botXYZ[0] < 0.075)) || enabled) {
//            Pose3d robotPose3d = new Pose3d(inputsLimelight.botXYZ[0], inputsLimelight.botXYZ[1], inputsLimelight.botXYZ[2], new Rotation3d(inputsLimelight.botRPY[0], inputsLimelight.botRPY[1], inputsLimelight.botRPY[2]));
////            exit if off the field
//            if (robotPose3d.getX() < -VisionConstants.FIELD_BORDER_MARGIN
//                    || robotPose3d.getX() > FieldConstants.fieldLength + VisionConstants.FIELD_BORDER_MARGIN
//                    || robotPose3d.getY() < -VisionConstants.FIELD_BORDER_MARGIN
//                    || robotPose3d.getY() > FieldConstants.fieldWidth + VisionConstants.FIELD_BORDER_MARGIN
//                    || robotPose3d.getZ() < -VisionConstants.Z_MARGIN
//                    || robotPose3d.getZ() > VisionConstants.Z_MARGIN) {
//            }
//            else {
//                Swerve.getInstance().addVisionData(getPose(), getTime(), checkStable());
//            }
//            }
//            Logger.getInstance().recordOutput("Vision/Pose", getPose());
//        }
//        Logger.getInstance().recordOutput("Vision/Stable", checkStable());
    }

//    https://github.com/Mechanical-Advantage/RobotCode2023/blob/main/src/main/java/org/littletonrobotics/frc2023/subsystems/apriltagvision/AprilTagVision.java
//    private double calculateAvgDistance() {
//        List<Pose3d> tagPoses = new ArrayList<>();
//        for (int i = (values[0] == 1 ? 9 : 17); i < values.length; i++) {
//            int tagId = (int) values[i];
//            lastTagDetectionTimes.put(tagId, Timer.getFPGATimestamp());
//            Optional<Pose3d> tagPose = FieldConstants.aprilTags.getTagPose((int) values[i]);
//            if (tagPose.isPresent()) {
//                tagPoses.add(tagPose.get());
//            }
//        }
//
//        // Calculate average distance to tag
//        double totalDistance = 0.0;
//        for (Pose3d tagPose : tagPoses) {
//            totalDistance += tagPose.getTranslation().getDistance(cameraPose.getTranslation());
//        }
//        double avgDistance = totalDistance / tagPoses.size();
//    }

    public void setDataInterface(Consumer<List<TimestampedVisionUpdate>> visionConsumer) {
        this.visionConsumer = visionConsumer;
    }

    public boolean checkStable() {
        if (getPose() != null && !getPose().equals(new Pose2d())) {
            if (GeomUtil.distance(getPose(), lastPose) < 0.1) {
                if (!timerStarted) {
                    timerStarted = true;
                    timer.restart();
                }
            }
            else {
                timerStarted = false;
                timer.restart();
                lastPose = getPose();
            }
        }
        else {
            timer.restart();
        }
        return timer.hasElapsed(0.2);
    }

    public boolean hasTarget() {
        return inputsLimelight.hasTarget || inputsPhotonLeft.hasTarget || inputsPhotonRight.hasTarget || inputsPhotonFrontLeft.hasTarget || inputsPhotonFrontRight.hasTarget;
    }

    @Deprecated
    public double getHorizontal() {
        return inputsLimelight.hAngle;
    }

    @Deprecated
    public double getVertical() {
        return inputsLimelight.vAngle;
    }

    private boolean checkRightLimelightHasPose() {
        return inputsLimelight.botXYZ.length != 0 && inputsLimelight.botRPY.length != 0;
    }

//    private boolean checkLeftLimelightHasPose() {
//        return inputsLeft.botXYZ.length != 0 && inputsLeft.botRPY.length != 0;
//    }

    public Pose2d getPose() {
//        if (checkRightLimelightHasPose() && checkLeftLimelightHasPose()) {
//            return new Pose2d(new Translation2d((inputsRight.botXYZ[0] + inputsLeft.botXYZ[0]) / 2, (inputsRight.botXYZ[1] + inputsLeft.botXYZ[1]) / 2), Rotation2d.fromDegrees(inputsRight.botRPY[2]));
//        }
        /*else */if (checkRightLimelightHasPose()) {
            return new Pose2d(new Translation2d(inputsLimelight.botXYZ[0], inputsLimelight.botXYZ[1]), Rotation2d.fromDegrees(inputsLimelight.botRPY[2]));
        }
//        else if (checkLeftLimelightHasPose()) {
//            return new Pose2d(new Translation2d(inputsLeft.botXYZ[0], inputsLeft.botXYZ[1]), Rotation2d.fromDegrees(inputsLeft.botRPY[2]));
//        }
        return null;
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public double getTime() {
        return inputsLimelight.captureTimestamp;
    }

    public void setPipeline(Pipelines pipeline) {
        this.pipeline = pipeline;
    }

//    public void setLED(LED led) {
//        this.led = led;
//    }

    public void setCamMode(CameraMode camera) {
        this.camera = camera;
    }

    public boolean isConnected() {
        return inputsLimelight.connected;
    }
}