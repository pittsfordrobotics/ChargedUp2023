package com.team3181.frc2023;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.team3181.frc2023.Constants.AutoConstants;

public class Paths {
    public static final PathPlannerTrajectory TEST = PathPlanner.loadPath("Test", AutoConstants.MAX_SPEED);

    public static final PathPlannerTrajectory DROP_CLIMB_HIGH = PathPlanner.loadPath("Drop and Climb High", AutoConstants.MAX_SPEED);

    public static final PathPlannerTrajectory DROP_CLIMB_LOW = PathPlanner.loadPath("Drop and Climb Low", AutoConstants.MAX_SPEED);
}