package com.team3181.frc2023.commands.autos;


import com.team3181.frc2023.Paths;
import com.team3181.frc2023.commands.EndEffectorRun;
import com.team3181.frc2023.commands.SuperstructureObjectiveGlobal;
import com.team3181.frc2023.commands.SwervePathing;
import com.team3181.frc2023.subsystems.objectivetracker.ObjectiveTracker.NodeLevel;
import com.team3181.frc2023.subsystems.objectivetracker.ObjectiveTracker.Objective;
import com.team3181.lib.commands.FollowPathWithLiveEvents;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoSwerveThree extends SequentialCommandGroup {
    public AutoSwerveThree(boolean top) {
        super(
                new ParallelCommandGroup(
                    new EndEffectorRun(),
                    new SuperstructureObjectiveGlobal(new Objective(8, NodeLevel.HIGH))
                ),
                new FollowPathWithLiveEvents(
                        new SwervePathing((top ? Paths.TOP_THREE : Paths.BOTTOM_THREE), true),
                        (top ? Paths.TOP_THREE : Paths.BOTTOM_THREE).getMarkers()
                )
        );
    }
}