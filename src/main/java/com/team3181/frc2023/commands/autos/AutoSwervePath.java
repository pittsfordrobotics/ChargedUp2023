package com.team3181.frc2023.commands.autos;


import com.pathplanner.lib.PathPlannerTrajectory;
import com.team3181.frc2023.commands.EndEffectorRun;
import com.team3181.frc2023.commands.SwervePathing;
import com.team3181.frc2023.subsystems.objectivetracker.ObjectiveTracker.Objective;
import com.team3181.lib.commands.FollowPathWithLiveEvents;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoSwervePath extends SequentialCommandGroup {
    public AutoSwervePath(PathPlannerTrajectory path, Objective objective) {
        super(
                new EndEffectorRun(),
//                new SuperstructureObjectiveGlobal(objective),
                new FollowPathWithLiveEvents(
                        new SwervePathing(path, true),
                        path.getMarkers()
                )
        );
    }
}