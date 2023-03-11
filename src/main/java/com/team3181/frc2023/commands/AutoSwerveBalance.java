package com.team3181.frc2023.commands;


import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.team3181.frc2023.Paths;
import com.team3181.lib.commands.FollowPathWithLiveEvents;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoSwerveBalance extends SequentialCommandGroup {
    public AutoSwerveBalance() {
        super(
                new FollowPathWithLiveEvents(
                    new SwervePathing(Paths.PIECE_BOTTOM, true),
                    Paths.PIECE_BOTTOM.getMarkers()
                )
        );
    }
}