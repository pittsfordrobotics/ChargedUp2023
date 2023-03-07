package com.team3181.frc2023.subsystems.vision;

import com.team3181.frc2023.Constants.RobotConstants;
import com.team3181.frc2023.subsystems.swerve.Swerve;
import com.team3181.frc2023.subsystems.vision.VisionIO.CameraMode;
import com.team3181.frc2023.subsystems.vision.VisionIO.LED;
import com.team3181.frc2023.subsystems.vision.VisionIO.Pipelines;
import com.team3181.lib.util.Alert;
import com.team3181.lib.util.Alert.AlertType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
    private final VisionIO io;
    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

    private boolean autoPipeline = false;
    private Pipelines pipeline = Pipelines.MID_RANGE;
    private LED led = LED.PIPELINE;
    private CameraMode camera = CameraMode.VISION_PROCESSING;

    private final Alert limelightAlert = new Alert("Limelight not detected! Vision will NOT work!", AlertType.ERROR);

    private static final Vision INSTANCE = new Vision(RobotConstants.VISION);
    public static Vision getInstance() {
        return INSTANCE;
    }

    private Vision(VisionIO io) {
        this.io = io;
        ShuffleboardTab visionTab = Shuffleboard.getTab("Vision");
        visionTab.addBoolean("Has Target", this::hasTarget);
        visionTab.addNumber("Horizontal", this::getHorizontal);
        visionTab.addNumber("Vertical", this::getVertical);
        visionTab.addNumber("LED", led::getNum);
        visionTab.addBoolean("Heartbeat", () -> inputs.connected);
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
            Swerve.getInstance().addVisionData(Vision.getInstance().getPose(), Vision.getInstance().getLatency());
            Logger.getInstance().recordOutput("Vision/Pose", Vision.getInstance().getPose());
        }
    }

    public void setAutoPipeline(boolean autoPipeline) {
        this.autoPipeline = autoPipeline;
    }

    public void pipelineSwitcher() {
        ChassisSpeeds chassisSpeeds = Swerve.getInstance().getChassisSpeeds();
        double robotSpeed = Math.sqrt(Math.pow(chassisSpeeds.vxMetersPerSecond, 2) + Math.pow(chassisSpeeds.vyMetersPerSecond, 2));
        double robotXPos = Swerve.getInstance().getPose().getX();

//        TODO: see if we should use speed
        if (robotXPos >= 8.3 - 1.2 && robotXPos < 8.3 + 1.2) {
            pipeline = Pipelines.FAR_RANGE;
        }
        else if ((robotXPos >= 4.9 && robotXPos < 8.3 - 1.2) || (robotXPos >= 8.3 + 1.2 && robotXPos < 11.7)) {
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
            return new Pose2d(new Translation2d(inputs.botXYZ[0], inputs.botXYZ[1]), new Rotation2d(inputs.botYPR[0]));
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