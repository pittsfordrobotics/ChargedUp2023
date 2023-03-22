package com.team3181.frc2023.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.team3181.frc2023.subsystems.swerve.Swerve;
import com.team3181.frc2023.subsystems.vision.Vision;


public class SwerveResetPoseAprilTag extends CommandBase {
    private final Swerve swerve = Swerve.getInstance();
    private final Vision vision = Vision.getInstance();
    private final Pose2d trajPose;

    public SwerveResetPoseAprilTag(Trajectory trajectory) {
        addRequirements(this.swerve, this.vision);
        trajPose = trajectory.getInitialPose();
    }

    @Override
    public void execute() {
        Pose2d visionPose = vision.getPose();
        if (visionPose == null) {
            swerve.resetPose(trajPose);
        }
        else {
            swerve.resetPose(visionPose);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}