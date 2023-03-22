package com.team3181.frc2023.commands;


import com.team3181.frc2023.Constants.AutoConstants.AutoDrivePosition;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SwerveAutoScore extends SequentialCommandGroup {
    public SwerveAutoScore() {
        super(
                new SwervePathingOnTheFly(AutoDrivePosition.NODE, false),
                new SuperstructureObjective()
        );
    }
}