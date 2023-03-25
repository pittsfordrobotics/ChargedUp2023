package com.team3181.frc2023.commands;


import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;

public class SwerveAutoDoubleSubstationLeft extends ParallelDeadlineGroup {
    public SwerveAutoDoubleSubstationLeft() {
        super(
                new SuperstructureDoubleSubstation(),
                new SwervePathingOnTheFly(true, true)
        );
    }
}