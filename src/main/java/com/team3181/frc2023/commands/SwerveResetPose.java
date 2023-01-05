package com.team3181.frc2023.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.team3181.frc2023.subsystems.swerve.Swerve;


public class SwerveResetPose extends CommandBase {
    private final Swerve swerve = Swerve.getInstance();
    private final Trajectory trajectory;
    private final boolean reset;

    public SwerveResetPose(Trajectory trajectory, boolean reset) {
        addRequirements(this.swerve);
        this.trajectory = trajectory;
        this.reset = reset;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (reset) {
            swerve.resetPose(trajectory.getInitialPose());
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {

    }
}