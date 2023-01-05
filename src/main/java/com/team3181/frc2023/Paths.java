package com.team3181.frc2023;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.team3181.frc2023.Constants.AutoConstants;

import java.util.ArrayList;

public class Paths {
    public static final PathPlannerTrajectory TEST = PathPlanner.loadPath("Test", AutoConstants.MAX_SPEED);
    public static final ArrayList<PathPlannerTrajectory> FIVE_BALL = PathPlanner.loadPathGroup("5 Ball", AutoConstants.MAX_SPEED);
}