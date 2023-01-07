package com.team3181.frc2023;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.team3181.frc2023.Constants.AutoConstants;

import java.util.List;

public class Paths {
    public static final PathPlannerTrajectory TEST = PathPlanner.loadPath("Test", AutoConstants.MAX_SPEED);
    public static final List<PathPlannerTrajectory> FIVE_BALL = PathPlanner.loadPathGroup("5 Ball", AutoConstants.MAX_SPEED);
}