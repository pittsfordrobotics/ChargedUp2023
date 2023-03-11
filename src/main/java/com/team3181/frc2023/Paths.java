package com.team3181.frc2023;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.team3181.frc2023.Constants.AutoConstants;
import com.team3181.frc2023.commands.*;
import com.team3181.frc2023.subsystems.Superstructure.GamePiece;
import com.team3181.frc2023.subsystems.objectivetracker.ObjectiveTracker.NodeLevel;
import com.team3181.frc2023.subsystems.objectivetracker.ObjectiveTracker.Objective;
import com.team3181.frc2023.subsystems.swerve.Swerve;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.util.HashMap;
import java.util.List;

public class Paths {
    public static final List<PathPlannerTrajectory> THREE_TOP = PathPlanner.loadPathGroup("Top 3 Item", AutoConstants.MAX_SPEED);
    public static final List<PathPlannerTrajectory> THREE_BOTTOM = PathPlanner.loadPathGroup("Bottom 3 Item", AutoConstants.MAX_SPEED);
    public static final PathPlannerTrajectory BOTTOM_CUBE = PathPlanner.loadPath("Bottom Cube", AutoConstants.MAX_SPEED);
    public static final PathPlannerTrajectory BOTTOM_CONE = PathPlanner.loadPath("Bottom Cone", AutoConstants.MAX_SPEED);
    public static final PathPlannerTrajectory BOTTOM_CUBE_PLUS_ONE = PathPlanner.loadPath("Bottom Cube + 1", AutoConstants.MAX_SPEED);
    public static final PathPlannerTrajectory BOTTOM_CONE_PLUS_ONE = PathPlanner.loadPath("Bottom Cone + 1", AutoConstants.MAX_SPEED);
    public static final PathPlannerTrajectory TOP_CUBE = PathPlanner.loadPath("Top Cube", AutoConstants.MAX_SPEED);
    public static final PathPlannerTrajectory TOP_CONE = PathPlanner.loadPath("Top Cone", AutoConstants.MAX_SPEED);
    public static final PathPlannerTrajectory TOP_CUBE_PLUS_ONE = PathPlanner.loadPath("Top Cube + 1", AutoConstants.MAX_SPEED);
    public static final PathPlannerTrajectory TOP_CONE_PLUS_ONE = PathPlanner.loadPath("Top Cone + 1", AutoConstants.MAX_SPEED);
    public static final PathPlannerTrajectory BALANCE = PathPlanner.loadPath("Balance", AutoConstants.MAX_SPEED);

    public static HashMap<String, Command> EVENT_MAP = new HashMap<>();
    public static final HashMap<String, Command> EVENT_MAP_BALANCE = new HashMap<>();
    public static final HashMap<String, Command> EVENT_MAP_NO_BALANCE = new HashMap<>();
    static {
        EVENT_MAP_BALANCE.put("intakeGround", new SuperstructureGround());
        EVENT_MAP_NO_BALANCE.put("intakeGround", new SuperstructureGround());

        EVENT_MAP_BALANCE.put("objectiveConeTop", new SuperstructureObjectiveGlobal(new Objective(6, NodeLevel.HIGH), GamePiece.CONE));
        EVENT_MAP_NO_BALANCE.put("objectiveConeTop", new SuperstructureObjectiveGlobal(new Objective(6, NodeLevel.HIGH), GamePiece.CONE));

        EVENT_MAP_BALANCE.put("objectiveCubeTop", new SuperstructureObjectiveGlobal(new Objective(7, NodeLevel.HIGH), GamePiece.CUBE));
        EVENT_MAP_NO_BALANCE.put("objectiveCubeTop", new SuperstructureObjectiveGlobal(new Objective(7, NodeLevel.HIGH), GamePiece.CUBE));

        EVENT_MAP_BALANCE.put("objectiveConeBottom", new SuperstructureObjectiveGlobal(new Objective(2, NodeLevel.HIGH), GamePiece.CONE));
        EVENT_MAP_NO_BALANCE.put("objectiveConeBottom", new SuperstructureObjectiveGlobal(new Objective(2, NodeLevel.HIGH), GamePiece.CONE));

        EVENT_MAP_BALANCE.put("objectiveCubeBottom", new SuperstructureObjectiveGlobal(new Objective(1, NodeLevel.HIGH), GamePiece.CUBE));
        EVENT_MAP_NO_BALANCE.put("objectiveCubeBottom", new SuperstructureObjectiveGlobal(new Objective(1, NodeLevel.HIGH), GamePiece.CUBE));

        EVENT_MAP_BALANCE.put("intakeGround", new SuperstructureGround());
        EVENT_MAP_NO_BALANCE.put("intakeGround", new SuperstructureGround());

        EVENT_MAP_BALANCE.put("home", new SuperstructureHome());
        EVENT_MAP_NO_BALANCE.put("home", new SuperstructureHome());

        EVENT_MAP_BALANCE.put("maybeStop", new WaitCommand(0));
        EVENT_MAP_NO_BALANCE.put("maybeStop", new SwerveStop());

        EVENT_MAP_BALANCE.put("balance", new SwerveAutoBalance());
        EVENT_MAP_NO_BALANCE.put("balance", new SwerveStop());
    }
}