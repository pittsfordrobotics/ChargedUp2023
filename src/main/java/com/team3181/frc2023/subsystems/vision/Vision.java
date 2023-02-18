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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
    private final VisionIO io;
    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

    private Pipelines pipeline = Pipelines.APRIL_TAGS;
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

        if (!RobotConstants.IS_TANK) {
            Swerve.getInstance().addVisionData(new Pose2d(inputs.botXYZ[0],inputs.botXYZ[1], Rotation2d.fromDegrees(inputs.botYPR[0])), inputs.captureTimestamp);
        }

        limelightAlert.set(!inputs.connected);

        io.setPipeline(pipeline);
        io.setCameraModes(camera);
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
        return new Pose2d();
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