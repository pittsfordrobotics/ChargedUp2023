package com.team3181.frc2023.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.team3181.frc2023.Constants.AutoConstants;
import com.team3181.frc2023.subsystems.swerve.Swerve;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class SwervePathing extends CommandBase {
    private final Swerve swerve = Swerve.getInstance();
    private final PathPlannerTrajectory trajectory;
    private final boolean reset;
    private final Timer timer = new Timer();

    private final PIDController xController = new PIDController(AutoConstants.LINEAR_P, 0, 0);
    private final PIDController yController = new PIDController(AutoConstants.LINEAR_P, 0, 0);
    private final ProfiledPIDController rotController = new ProfiledPIDController(AutoConstants.ROT_P, 0, 0, AutoConstants.MAX_ROT_CONSTRAINTS);
    private final HolonomicDriveController holonomicDriveController = new HolonomicDriveController(xController, yController, rotController);

    public SwervePathing(PathPlannerTrajectory trajectory, boolean reset) {
        addRequirements(this.swerve);
        this.trajectory = trajectory;
        this.reset = reset;
        rotController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        // adjusts because the field is flipped instead of rotated by 180
        PathPlannerState adjustedState = PathPlannerTrajectory.transformStateForAlliance(trajectory.getInitialState(), DriverStation.getAlliance());
        if (reset) {
            swerve.resetPose(new Pose2d(adjustedState.poseMeters.getTranslation(), adjustedState.holonomicRotation));
        }
        xController.reset();
        yController.reset();
        rotController.reset(adjustedState.holonomicRotation.getRadians());

        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        PathPlannerState state = (PathPlannerState) trajectory.sample(timer.get());
        PathPlannerState adjustedState = PathPlannerTrajectory.transformStateForAlliance(state, DriverStation.getAlliance());
        ChassisSpeeds speeds = holonomicDriveController.calculate(swerve.getPose(), adjustedState, adjustedState.holonomicRotation);
        swerve.setChassisSpeeds(speeds, false);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        swerve.stopMotors();
    }
}