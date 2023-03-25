package com.team3181.frc2023.commands;


import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;

public class SwerveAutoDoubleSubstationRight extends ParallelDeadlineGroup {
    public SwerveAutoDoubleSubstationRight() {
        super(
                new SuperstructureDoubleSubstation(),
                new SwervePathingOnTheFly(false, true)
        );
    }
}