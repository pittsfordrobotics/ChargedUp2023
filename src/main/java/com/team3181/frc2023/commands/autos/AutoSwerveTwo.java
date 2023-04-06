package com.team3181.frc2023.commands.autos;


import com.team3181.frc2023.Paths;
import com.team3181.frc2023.commands.EndEffectorRun;
import com.team3181.frc2023.commands.SuperstructureObjectiveGlobal;
import com.team3181.frc2023.commands.SwerveAutoBalance;
import com.team3181.frc2023.commands.SwervePathing;
import com.team3181.frc2023.subsystems.objectivetracker.ObjectiveTracker.NodeLevel;
import com.team3181.frc2023.subsystems.objectivetracker.ObjectiveTracker.Objective;
import com.team3181.lib.commands.FollowPathWithLiveEvents;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoSwerveTwo extends SequentialCommandGroup {
    public AutoSwerveTwo(boolean top) {
        super(
                new ParallelCommandGroup(
                    new EndEffectorRun(),
                    new SuperstructureObjectiveGlobal(new Objective(8, NodeLevel.HIGH))
                ),
                new FollowPathWithLiveEvents(
                        new SwervePathing((top ? Paths.TOP_TWO_PLUS_ONE : Paths.BOTTOM_TWO_PLUS_ONE), true),
                        (top ? Paths.TOP_TWO_PLUS_ONE : Paths.BOTTOM_TWO_PLUS_ONE).getMarkers()
                ),
                new SwerveAutoBalance(false)
        );
    }
}