package com.team3181.frc2023;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.team3181.frc2023.Constants.AutoConstants;

import java.util.List;

public class Paths {
    public static final PathPlannerTrajectory DROP_CLIMB_HIGH = PathPlanner.loadPath("Drop and Climb High", AutoConstants.MAX_SPEED);
    public static final PathPlannerTrajectory DROP_CLIMB_LOW = PathPlanner.loadPath("Drop and Climb Low", AutoConstants.MAX_SPEED);

    public static final List<PathPlannerTrajectory> PIECE_AUTO_HIGH = PathPlanner.loadPathGroup("Drop and Climb High", AutoConstants.MAX_SPEED);
    public static final List<PathPlannerTrajectory>  PIECE_AUTO_LOW = PathPlanner.loadPathGroup("Drop and Climb Low", AutoConstants.MAX_SPEED);
}