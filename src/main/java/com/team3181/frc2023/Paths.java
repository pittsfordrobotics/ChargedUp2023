package com.team3181.frc2023;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.team3181.frc2023.Constants.AutoConstants;
import com.team3181.frc2023.subsystems.swerve.Swerve;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.List;

public class Paths {
    public static final PathPlannerTrajectory TEST_ON_THE_FLY = PathPlanner.generatePath(
            AutoConstants.MAX_SPEED,
                new PathPoint(Swerve.getInstance().getPose().getTranslation(), Swerve.getInstance().getPose().getRotation(), Swerve.getInstance().getPose().getRotation()),
                new PathPoint(new Translation2d(5,1), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0))
            );

    public static final List<PathPlannerTrajectory> PIECE_AUTO_HIGH = PathPlanner.loadPathGroup("3 Item High", AutoConstants.MAX_SPEED);
    public static final PathPlannerTrajectory PIECE_AUTO_HIGH_NOTGROUP = PathPlanner.loadPath("3 Item Low", AutoConstants.MAX_SPEED);
    public static final List<PathPlannerTrajectory>  PIECE_AUTO_LOW = PathPlanner.loadPathGroup("3 Item Low", AutoConstants.MAX_SPEED);
}