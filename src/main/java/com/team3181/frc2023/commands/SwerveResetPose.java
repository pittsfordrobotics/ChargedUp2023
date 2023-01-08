package com.team3181.frc2023.commands;

import com.team3181.frc2023.subsystems.swerve.Swerve;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class SwerveResetPose extends CommandBase {
    private final Swerve swerve = Swerve.getInstance();
    private final Pose2d pose;

    public SwerveResetPose(Trajectory trajectory) {
        addRequirements(this.swerve);
        this.pose = trajectory.getInitialPose();
    }

    public SwerveResetPose(Pose2d pose) {
        addRequirements(this.swerve);
        this.pose = pose;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        swerve.resetPose(pose);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {

    }
}