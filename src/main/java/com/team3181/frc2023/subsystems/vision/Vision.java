package com.team3181.frc2023.subsystems.vision;

import com.team3181.frc2023.Constants.RobotConstants;
import com.team3181.frc2023.subsystems.swerve.Swerve;
import com.team3181.frc2023.subsystems.vision.VisionIO.CameraMode;
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

import java.util.Objects;

public class Vision extends SubsystemBase {
    private final VisionIO ioRight;
    private final VisionIO ioLeft;
    private final VisionIOInputsAutoLogged inputsRight = new VisionIOInputsAutoLogged();
    private final VisionIOInputsAutoLogged inputsLeft = new VisionIOInputsAutoLogged();

    private boolean autoPipeline = false;
    private Pipelines pipeline = Pipelines.MID_RANGE;
//    private LED led = LED.PIPELINE;
    private CameraMode camera = CameraMode.VISION_PROCESSING;
    private final Timer timer = new Timer();
    private boolean timerStarted = false;
    private boolean enabled = true;
    private Pose2d lastPose = new Pose2d();

    private final Alert rightLimelightAlert = new Alert("Right Limelight not detected! Vision will NOT work!", AlertType.ERROR);
    private final Alert leftLimelightAlert = new Alert("Left Limelight not detected! Vision will NOT work!", AlertType.ERROR);

    private static final Vision INSTANCE = new Vision(RobotConstants.VISION_RIGHT, RobotConstants.VISION_LEFT);
    public static Vision getInstance() {
        return INSTANCE;
    }

    // right and left are relative to the robot from birds eye view facing forwards
    private Vision(VisionIO ioRight, VisionIO ioLeft) {
        this.ioRight = ioRight;
        this.ioLeft = ioLeft;
    }

    @Override
    public void periodic() {
        ioRight.updateInputs(inputsRight);
        Logger.getInstance().processInputs("VisionRight", inputsRight);
        Logger.getInstance().processInputs("VisionLeft", inputsLeft);

        rightLimelightAlert.set(!inputsRight.connected);
        leftLimelightAlert.set(!inputsLeft.connected);

        if (!DriverStation.isAutonomous() && autoPipeline) {
            pipelineSwitcher();
        }

        ioRight.setPipeline(pipeline);
        ioRight.setCameraModes(camera);
        ioLeft.setPipeline(pipeline);
        ioLeft.setCameraModes(camera);
        Logger.getInstance().recordOutput("Vision/Pipeline", pipeline.toString());
        Logger.getInstance().recordOutput("Vision/Camera", camera.toString());
        Logger.getInstance().recordOutput("Vision/Enabled", enabled);

        if (!Objects.equals(getPose(), new Pose2d()) && getPose() != null && enabled && !DriverStation.isAutonomous()) {
            Swerve.getInstance().addVisionData(getPose(), getTime(), checkStable());
        }
        Logger.getInstance().recordOutput("Vision/Pose", getPose());
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
        return timer.hasElapsed(0.05);
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
        return inputsRight.hasTarget;
    }

    public double getHorizontal() {
        return inputsRight.hAngle;
    }

    public double getVertical() {
        return inputsRight.vAngle;
    }

    private boolean checkRightLimelightHasPose() {
        return inputsRight.botXYZ.length != 0 && inputsRight.botRPY.length != 0;
    }

    private boolean checkLeftLimelightHasPose() {
        return inputsLeft.botXYZ.length != 0 && inputsLeft.botRPY.length != 0;
    }

    public Pose2d getPose() {
        if (checkRightLimelightHasPose() && checkLeftLimelightHasPose()) {
            return new Pose2d(new Translation2d((inputsRight.botXYZ[0] + inputsLeft.botXYZ[0]) / 2, (inputsRight.botXYZ[1] + inputsLeft.botXYZ[1]) / 2), Rotation2d.fromDegrees(inputsRight.botRPY[2]));
        }
        else if (checkRightLimelightHasPose()) {
            return new Pose2d(new Translation2d(inputsRight.botXYZ[0], inputsRight.botXYZ[1]), Rotation2d.fromDegrees(inputsRight.botRPY[2]));
        }
        else if (checkLeftLimelightHasPose()) {
            return new Pose2d(new Translation2d(inputsLeft.botXYZ[0], inputsLeft.botXYZ[1]), Rotation2d.fromDegrees(inputsLeft.botRPY[2]));
        }
        return null;
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public double getTime() {
        return inputsRight.captureTimestamp;
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
        return inputsRight.connected;
    }
}