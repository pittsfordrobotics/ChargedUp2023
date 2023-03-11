package com.team3181.frc2023.commands;


import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.team3181.frc2023.Paths;
import com.team3181.frc2023.subsystems.Superstructure;
import com.team3181.frc2023.subsystems.Superstructure.GamePiece;
import com.team3181.frc2023.subsystems.objectivetracker.ObjectiveTracker.NodeLevel;
import com.team3181.frc2023.subsystems.objectivetracker.ObjectiveTracker.Objective;
import edu.wpi.first.wpilibj2.command.*;

public class AutoSwerveHighThree extends SequentialCommandGroup {
    public AutoSwerveHighThree() {
        super(
//                new SuperstructureObjectiveGlobal(new Objective(8, NodeLevel.HIGH), GamePiece.CONE),
//                new FollowPathWithEvents(
//                        getPathFollowingCommand(examplePath),
//                        examplePath.getMarkers(),
//                        eventMap
//                );

        );
    }
}