package com.team3181.frc2023.commands.autos;


import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.team3181.frc2023.Paths;
import com.team3181.frc2023.Robot;
import com.team3181.frc2023.RobotContainer;
import com.team3181.frc2023.commands.SuperstructureObjective;
import com.team3181.frc2023.commands.SwervePathing;
import com.team3181.frc2023.subsystems.objectivetracker.ObjectiveTracker.Objective;
import com.team3181.lib.commands.FollowPathWithLiveEvents;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoSwerveBalance extends SequentialCommandGroup {
    public AutoSwerveBalance() {
        super(
                new SuperstructureObjective(),
                new FollowPathWithLiveEvents(
                    new SwervePathing(Paths.BALANCE, true),
                    Paths.BALANCE.getMarkers()
                )
        );
    }
}