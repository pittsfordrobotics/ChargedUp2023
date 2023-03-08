package com.team3181.frc2023.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.PathPoint;
import com.team3181.frc2023.Constants.AutoConstants;
import com.team3181.frc2023.Constants.AutoConstants.AutoDrivePosition;
import com.team3181.frc2023.FieldConstants.AutoDrivePoints;
import com.team3181.frc2023.subsystems.objectivetracker.ObjectiveTracker;
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
    private final AutoDrivePosition position;
    private final boolean simple;
    private final Timer timer = new Timer();
    private PathConstraints pathConstraints;

    private final PIDController xController = new PIDController(AutoConstants.LINEAR_P, 0, 0);
    private final PIDController yController = new PIDController(AutoConstants.LINEAR_P, 0, 0);
    private final ProfiledPIDController rotController = new ProfiledPIDController(AutoConstants.ROT_P, 0, 0, AutoConstants.MAX_ROT_CONSTRAINTS);
    private final HolonomicDriveController holonomicDriveController = new HolonomicDriveController(xController, yController, rotController);

    public SwervePathingOnTheFly(BetterPathPoint... pathPoint) {
        addRequirements(this.swerve);
        this.pathPoint = pathPoint;
        pathConstraints = AutoConstants.MAX_SPEED;
        simple = false;
        position = null;
        rotController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public SwervePathingOnTheFly(PathConstraints pathConstraints, BetterPathPoint... pathPoint) {
        addRequirements(this.swerve);
        this.pathPoint = pathPoint;
        this.pathConstraints = pathConstraints;
        simple = true;
        position = null;
        rotController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public SwervePathingOnTheFly(AutoDrivePosition position, boolean simple) {
        addRequirements(this.swerve);
        this.pathPoint = null;
        this.pathConstraints = AutoConstants.MAX_SPEED;
        this.simple = simple;
        this.position = position;
        rotController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        ArrayList<PathPoint> adjustedPathPoints = new ArrayList<>();
        // adds path points and flips for correct alliance
        BetterPathPoint robotPoint = new BetterPathPoint(Swerve.getInstance().getPose().getTranslation(), Swerve.getInstance().getPose().getRotation(), Swerve.getInstance().getPose().getRotation());
//        BetterPathPoint robotPoint = new BetterPathPoint(Swerve.getInstance().getPose().getTranslation(), Swerve.getInstance().getPose().getRotation(), Swerve.getInstance().getPose().getRotation());
        switch (position) {
            case NODE:
                BetterPathPoint node = AutoDrivePoints.nodeSelector(ObjectiveTracker.getInstance().getObjective().nodeRow);
                if (simple) {
                    BetterPathPoint headingCorrection = AutoDrivePoints.updateHeading(robotPoint, AutoDrivePoints.pathPointFlipper(node, DriverStation.getAlliance()));
                    adjustedPathPoints.add(headingCorrection);
                    adjustedPathPoints.add(AutoDrivePoints.pathPointFlipper(node, DriverStation.getAlliance()));
                }
                else {
//                    if
                }
                break;
            case SINGLE_SUBSTATION:
//                TODO: this won't be implemented for right now
                break;
            case DOUBLE_SUBSTATION_HIGH:
                if (simple) {
                    BetterPathPoint headingCorrection = AutoDrivePoints.updateHeading(robotPoint, AutoDrivePoints.pathPointFlipper(AutoDrivePoints.LOADING_STATION_TOP_INNER, DriverStation.getAlliance()));
                    adjustedPathPoints.add(headingCorrection);
                    adjustedPathPoints.add(AutoDrivePoints.pathPointFlipper(AutoDrivePoints.LOADING_STATION_TOP_INNER, DriverStation.getAlliance()));
                }
                break;
            case DOUBLE_SUBSTATION_LOW:
                if (simple) {
                    BetterPathPoint headingCorrection = AutoDrivePoints.updateHeading(robotPoint, AutoDrivePoints.pathPointFlipper(AutoDrivePoints.LOADING_STATION_BOTTOM_INNER, DriverStation.getAlliance()));
                    adjustedPathPoints.add(headingCorrection);
                    adjustedPathPoints.add(AutoDrivePoints.pathPointFlipper(AutoDrivePoints.LOADING_STATION_BOTTOM_INNER, DriverStation.getAlliance()));
                }
                break;
            default:
                for (int i = 1; i < pathPoint.length + 1; i++) {
                    if (i == 1) {
                        BetterPathPoint headingCorrection = AutoDrivePoints.updateHeading(robotPoint, AutoDrivePoints.pathPointFlipper(pathPoint[i - 1], DriverStation.getAlliance()));
                        adjustedPathPoints.add(headingCorrection);
                    }
                    adjustedPathPoints.add(AutoDrivePoints.pathPointFlipper(pathPoint[i - 1], DriverStation.getAlliance()));
                }
                break;
        }
        // creates trajectory
        try {
            this.trajectory = PathPlanner.generatePath(
                    pathConstraints,
                    adjustedPathPoints
            );
        } catch (Exception ignored) {}
        // path following setup
        xController.reset();
        yController.reset();
        rotController.reset(trajectory.getInitialState().holonomicRotation.getRadians());

        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        PathPlannerState state = (PathPlannerState) trajectory.sample(timer.get());
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