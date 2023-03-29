package com.team3181.frc2023.commands.autos;


import com.team3181.frc2023.Paths;
import com.team3181.frc2023.RobotContainer;
import com.team3181.frc2023.commands.*;
import com.team3181.frc2023.subsystems.objectivetracker.ObjectiveTracker.NodeLevel;
import com.team3181.frc2023.subsystems.objectivetracker.ObjectiveTracker.Objective;
import com.team3181.lib.commands.FollowPathWithLiveEvents;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoSwerveThreeBalance extends SequentialCommandGroup {
    public AutoSwerveThreeBalance(boolean top) {
        super(
                new EndEffectorRun(),
                new SwerveResetPoseAprilTag(),
                new SuperstructureObjectiveGlobalDemand(new Objective(8, NodeLevel.HIGH)),
                new SwerveAutoScore(),
//                new FollowPathWithLiveEvents(
//                        new SwervePathing((top ? Paths.TOP_THREE_BALANCE : Paths.BOTTOM_THREE), true),
//                        (top ? Paths.TOP_THREE_BALANCE : Paths.BOTTOM_THREE).getMarkers()
//                ),
                new FollowPathWithLiveEvents(
                        new SwervePathing((top ? Paths.TOP_THREE_BALANCE : Paths.BOTTOM_THREE), true),
                        (top ? Paths.TOP_THREE_BALANCE : Paths.BOTTOM_THREE).getMarkers()
                ),
                new WaitCommand(0.5),
                new SwerveAutoBalance(RobotContainer.balanceForward)
        );
    }
}