package com.team3181.frc2023.commands;


import com.team3181.frc2023.Constants.AutoConstants;
import com.team3181.frc2023.FieldConstants.AutoDrivePoints;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SwerveAutoDoubleSubstationLow extends ParallelDeadlineGroup {
    public SwerveAutoDoubleSubstationLow() {
        super(
                new SuperstructureDoubleSubstation(),
                new SwervePathingOnTheFly(AutoConstants.SLOW_SPEED, AutoDrivePoints.LOADING_STATION_BOTTOM_INNER)
        );
    }
}