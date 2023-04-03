package com.team3181.frc2023.commands;


import com.team3181.frc2023.subsystems.Superstructure;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;

public class SwerveAutoDoubleSubstationLeft extends ParallelDeadlineGroup {
    public SwerveAutoDoubleSubstationLeft() {
        super(
                new SuperstructureDoubleSubstation(),
                new InstantCommand(() -> Superstructure.getInstance().setAutoSubstation(true)),
                new SwervePathingOnTheFly(true, true),
                new InstantCommand(() -> Superstructure.getInstance().setAutoSubstation(false))
        );
    }
}