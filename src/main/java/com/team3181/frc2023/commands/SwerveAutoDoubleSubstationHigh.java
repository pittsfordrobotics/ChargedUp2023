package com.team3181.frc2023.commands;


import com.team3181.frc2023.Constants.AutoConstants;
import com.team3181.frc2023.Constants.AutoConstants.AutoDrivePosition;
import com.team3181.frc2023.FieldConstants.AutoDrivePoints;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SwerveAutoDoubleSubstationHigh extends ParallelDeadlineGroup {
    public SwerveAutoDoubleSubstationHigh() {
        super(
                new SuperstructureDoubleSubstation(),
                new SwervePathingOnTheFly(AutoDrivePosition.DOUBLE_SUBSTATION_HIGH, true)
        );
    }
}