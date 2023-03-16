package com.team3181.frc2023.subsystems.vision;

import com.team3181.frc2023.Constants.RobotConstants;
import com.team3181.frc2023.subsystems.swerve.Swerve;
import com.team3181.frc2023.subsystems.vision.VisionIO.CameraMode;
import com.team3181.frc2023.subsystems.vision.VisionIO.LED;
import com.team3181.frc2023.subsystems.vision.VisionIO.Pipelines;
import com.team3181.lib.math.GeomUtil;
import com.team3181.lib.util.Alert;
import com.team3181.lib.util.Alert.AlertType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
    private final VisionIO io;
    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

    private boolean autoPipeline = false;
    private Pipelines pipeline = Pipelines.MID_RANGE;
    private LED led = LED.PIPELINE;
    private CameraMode camera = CameraMode.VISION_PROCESSING;
    private final Timer timer = new Timer();
    private boolean timerStarted = false;
    private Pose2d lastPose = new Pose2d();

    private final Alert limelightAlert = new Alert("Limelight not detected! Vision will NOT work!", AlertType.ERROR);

    private static final Vision INSTANCE = new Vision(RobotConstants.VISION);
    public static Vision getInstance() {
        return INSTANCE;
    }

    private Vision(VisionIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Vision", inputs);

        limelightAlert.set(!inputs.connected);

        if (!DriverStation.isAutonomous() && autoPipeline) {
            pipelineSwitcher();
        }

        io.setPipeline(pipeline);
        io.setCameraModes(camera);
        Logger.getInstance().recordOutput("Vision/Pipeline", pipeline.toString());
        Logger.getInstance().recordOutput("Vision/Camera", camera.toString());

        if (getPose() != null) {
            Swerve.getInstance().addVisionData(getPose(), getLatency(), checkStable());
            Logger.getInstance().recordOutput("Vision/Pose", getPose());
        }
        Logger.getInstance().recordOutput("Vision/Stable", checkStable());
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
        return timer.hasElapsed(0.5);
    }

    public void setAutoPipeline(boolean autoPipeline) {
        this.autoPipeline = autoPipeline;
    }

    public void pipelineSwitcher() {
        ChassisSpeeds chassisSpeeds = Swerve.getInstance().getChassisSpeeds();
        double robotSpeed = Math.sqrt(Math.pow(chassisSpeeds.vxMetersPerSecond, 2) + Math.pow(chassisSpeeds.vyMetersPerSecond, 2));
        double robotXPos = Swerve.getInstance().getPose().getX();

//        TODO: see if we should use speed
        if ((robotXPos >= 4.9 && robotXPos < 11.7)) {
            pipeline = Pipelines.MID_RANGE;
        }
        else if ((robotXPos >= 0 && robotXPos < 4.9) || (robotXPos >= 11.7 && robotXPos < 16.5)) {
            pipeline = Pipelines.CLOSE_RANGE;
        }
    }

    public boolean hasTarget() {
        return inputs.hasTarget;
    }

    public double getHorizontal() {
        return inputs.hAngle;
    }

    public double getVertical() {
        return inputs.vAngle;
    }

    public Pose2d getPose() {
        if (inputs.botXYZ.length != 0 && inputs.botYPR.length != 0) {
//            return new Pose2d(new Translation2d(inputs.botXYZ[0], inputs.botXYZ[1]), new Rotation2d(inputs.botYPR[2]));
            return new Pose2d(new Translation2d(inputs.botXYZ[0], inputs.botXYZ[1]), new Rotation2d());
        }
        return null;
    }

    public double getLatency() {
        return inputs.captureTimestamp;
    }

    public void setPipeline(Pipelines pipeline) {
        this.pipeline = pipeline;
    }

    public void setLED(LED led) {
        this.led = led;
    }

    public void setCamMode(CameraMode camera) {
        this.camera = camera;
    }

    public boolean isConnected() {
        return inputs.connected;
    }
}