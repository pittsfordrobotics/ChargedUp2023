package com.team3181.frc2023;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.team3181.frc2023.Constants.AutoConstants;
import com.team3181.frc2023.commands.*;
import com.team3181.frc2023.subsystems.objectivetracker.ObjectiveTracker.NodeLevel;
import com.team3181.frc2023.subsystems.objectivetracker.ObjectiveTracker.Objective;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.util.HashMap;

public class Paths {
    public static final PathPlannerTrajectory TOP_THREE = PathPlanner.loadPath("Top 3", AutoConstants.MAX_SPEED);
    public static final PathPlannerTrajectory NEW_PATH = PathPlanner.loadPath("New Path", AutoConstants.MAX_SPEED);
    public static final PathPlannerTrajectory BOTTOM_THREE = PathPlanner.loadPath("Bottom 3", AutoConstants.MAX_SPEED);
    public static final PathPlannerTrajectory TOP_TWO_PLUS_ONE = PathPlanner.loadPath("Top 2 + 1", AutoConstants.MAX_SPEED);
    public static final PathPlannerTrajectory BOTTOM_TWO_PLUS_ONE = PathPlanner.loadPath("Bottom 2 + 1", AutoConstants.MAX_SPEED);
    public static final PathPlannerTrajectory BOTTOM_CUBE_PLUS_ONE = PathPlanner.loadPath("Bottom Cube + 1", AutoConstants.SLOW_SPEED);
    public static final PathPlannerTrajectory BOTTOM_CONE_PLUS_ONE = PathPlanner.loadPath("Bottom Cone + 1", AutoConstants.SLOW_SPEED);
    public static final PathPlannerTrajectory TOP_CUBE_PLUS_ONE = PathPlanner.loadPath("Top Cube + 1", AutoConstants.SLOW_SPEED);
    public static final PathPlannerTrajectory TOP_CONE_PLUS_ONE = PathPlanner.loadPath("Top Cone + 1", AutoConstants.SLOW_SPEED);
    public static final PathPlannerTrajectory BALANCE = PathPlanner.loadPath("Balance", AutoConstants.SLOW_SPEED);

    public static HashMap<String, Command> EVENT_MAP = new HashMap<>();
    public static final HashMap<String, Command> EVENT_MAP_BALANCE = new HashMap<>();
    public static final HashMap<String, Command> EVENT_MAP_NO_BALANCE = new HashMap<>();
    static {
        EVENT_MAP_BALANCE.put("intakeGroundNo", new SuperstructureGround());
        EVENT_MAP_NO_BALANCE.put("intakeGroundNo", new SuperstructureGround());

        EVENT_MAP_BALANCE.put("intakeGround", new SuperstructureGroundFar());
        EVENT_MAP_NO_BALANCE.put("intakeGround", new SuperstructureGroundFar());

        EVENT_MAP_BALANCE.put("objectiveHybridTop", new SuperstructureObjectiveGlobal(new Objective(8, NodeLevel.HYBRID)));
        EVENT_MAP_NO_BALANCE.put("objectiveHybridTop", new SuperstructureObjectiveGlobal(new Objective(8, NodeLevel.HYBRID)));

        EVENT_MAP_BALANCE.put("objectiveHybridBottom", new SuperstructureObjectiveGlobal(new Objective(0, NodeLevel.HYBRID)));
        EVENT_MAP_NO_BALANCE.put("objectiveHybridBottom", new SuperstructureObjectiveGlobal(new Objective(0, NodeLevel.HYBRID)));

        EVENT_MAP_BALANCE.put("objectiveConeTop", new SuperstructureObjectiveGlobal(new Objective(6, NodeLevel.HIGH)));
        EVENT_MAP_NO_BALANCE.put("objectiveConeTop", new SuperstructureObjectiveGlobal(new Objective(6, NodeLevel.HIGH)));

        EVENT_MAP_BALANCE.put("objectiveCubeTop", new SuperstructureObjectiveGlobal(new Objective(7, NodeLevel.HIGH)));
        EVENT_MAP_NO_BALANCE.put("objectiveCubeTop", new SuperstructureObjectiveGlobal(new Objective(7, NodeLevel.HIGH)));

        EVENT_MAP_BALANCE.put("objectiveConeBottom", new SuperstructureObjectiveGlobal(new Objective(2, NodeLevel.HIGH)));
        EVENT_MAP_NO_BALANCE.put("objectiveConeBottom", new SuperstructureObjectiveGlobal(new Objective(2, NodeLevel.HIGH)));

        EVENT_MAP_BALANCE.put("objectiveCubeBottom", new SuperstructureObjectiveGlobal(new Objective(1, NodeLevel.HIGH)));
        EVENT_MAP_NO_BALANCE.put("objectiveCubeBottom", new SuperstructureObjectiveGlobal(new Objective(1, NodeLevel.HIGH)));

        EVENT_MAP_BALANCE.put("home", new SuperstructureHome());
        EVENT_MAP_NO_BALANCE.put("home", new SuperstructureHome());

        EVENT_MAP_BALANCE.put("maybeStop", new WaitCommand(0));
        EVENT_MAP_NO_BALANCE.put("maybeStop", new SwerveStop());
    }
}