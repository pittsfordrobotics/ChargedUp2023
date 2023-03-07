package com.team3181.frc2023.commands;


import com.team3181.frc2023.FieldConstants.AutoDrivePoints;
import com.team3181.frc2023.subsystems.objectivetracker.ObjectiveTracker;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SwerveAutoScore extends SequentialCommandGroup {
    public SwerveAutoScore() {
        super(
                new SwervePathingOnTheFly(AutoDrivePoints.nodeSelector(ObjectiveTracker.getInstance().getObjective().nodeRow)),
                new SuperstructureObjective()
        );
    }
}