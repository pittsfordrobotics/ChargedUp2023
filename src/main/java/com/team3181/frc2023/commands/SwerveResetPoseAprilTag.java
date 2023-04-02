package com.team3181.frc2023.commands;

import com.team3181.frc2023.subsystems.swerve.Swerve;
import com.team3181.frc2023.subsystems.vision.Vision;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class SwerveResetPoseAprilTag extends CommandBase {
    private final Swerve swerve = Swerve.getInstance();
    private final Vision vision = Vision.getInstance();

    public SwerveResetPoseAprilTag() {
        addRequirements(this.swerve, this.vision);
    }

    @Override
    public void execute() {
        Pose2d visionPose = swerve.getPose();

        if (visionPose != null) {
            swerve.resetPose(visionPose);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}