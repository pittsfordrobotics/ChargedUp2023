package com.team3181.frc2023.commands;


import com.team3181.frc2023.Constants.AutoConstants.AutoDrivePosition;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;

public class SwerveAutoDoubleSubstationLow extends ParallelDeadlineGroup {
    public SwerveAutoDoubleSubstationLow() {
        super(
                new SuperstructureDoubleSubstation(),
                new SwervePathingOnTheFly(AutoDrivePosition.DOUBLE_SUBSTATION_LOW, true)
        );
    }
}