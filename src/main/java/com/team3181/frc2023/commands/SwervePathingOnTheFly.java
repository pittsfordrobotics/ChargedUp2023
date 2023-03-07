package com.team3181.frc2023.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.PathPoint;
import com.team3181.frc2023.Constants.AutoConstants;
import com.team3181.frc2023.FieldConstants.AutoDrivePoints;
import com.team3181.frc2023.subsystems.swerve.Swerve;
import com.team3181.lib.swerve.BetterPathPoint;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.ArrayList;


public class SwervePathingOnTheFly extends CommandBase {
    private final Swerve swerve = Swerve.getInstance();
    private PathPlannerTrajectory trajectory;
    private final BetterPathPoint[] pathPoint;
    private final Timer timer = new Timer();
    private PathConstraints pathConstraints;

    private final PIDController xController = new PIDController(AutoConstants.LINEAR_P, 0, 0);
    private final PIDController yController = new PIDController(AutoConstants.LINEAR_P, 0, 0);
    private final ProfiledPIDController rotController = new ProfiledPIDController(AutoConstants.ROT_P, 0, 0, AutoConstants.MAX_ROT_CONSTRAINTS);
    private final HolonomicDriveController holonomicDriveController = new HolonomicDriveController(xController, yController, rotController);

    public SwervePathingOnTheFly(BetterPathPoint... pathPoint) {
        addRequirements(this.swerve);
        this.pathPoint = pathPoint;
        rotController.enableContinuousInput(-Math.PI, Math.PI);
        pathConstraints = AutoConstants.MAX_SPEED;
    }

    public SwervePathingOnTheFly(PathConstraints pathConstraints, BetterPathPoint... pathPoint) {
        addRequirements(this.swerve);
        this.pathPoint = pathPoint;
        this.pathConstraints = pathConstraints;
        rotController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        // adjusts because the field is flipped instead of rotated by 180
        ArrayList<PathPoint> adjustedPathPoints = new ArrayList<>();
        adjustedPathPoints.add(new PathPoint(Swerve.getInstance().getPose().getTranslation(), Swerve.getInstance().getPose().getRotation(), Swerve.getInstance().getPose().getRotation()));
        for (int i = 1; i < pathPoint.length + 1; i++) {
            adjustedPathPoints.add(AutoDrivePoints.pathPointFlipper(pathPoint[i-1], DriverStation.getAlliance()));
        }
        this.trajectory = PathPlanner.generatePath(
                pathConstraints,
                adjustedPathPoints
        );
//        PathPlannerState adjustedState = PathPlannerTrajectory.transformStateForAlliance(trajectory.getInitialState(), DriverStation.getAlliance());
        xController.reset();
        yController.reset();
        rotController.reset(trajectory.getInitialState().holonomicRotation.getRadians());

        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        PathPlannerState state = (PathPlannerState) trajectory.sample(timer.get());
//        PathPlannerState adjustedState = PathPlannerTrajectory.transformStateForAlliance(state, DriverStation.getAlliance());
        ChassisSpeeds speeds = holonomicDriveController.calculate(swerve.getPose(), state, state.holonomicRotation);
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